// Featherstone's Articulated-Body Algorithm: forward dynamics, generalized to
// joints of any DOF count (1 for revolute/prismatic, 6 for a free/floating base).
//
// Given joint positions q, velocities qd, and applied torques tau, compute the
// joint accelerations qdd in O(n) without forming the mass matrix. Gravity is
// injected as a fictitious base acceleration (the standard Featherstone trick).

import {
  v6, m66, mulMV, mulMtV, crossMotion, crossForce, congruence,
} from "./spatial.js";

const scratch = { n: 0 };
function ensure(n) {
  if (scratch.n >= n) return;
  scratch.Xup = Array.from({ length: n }, m66);
  scratch.v = Array.from({ length: n }, v6);
  scratch.c = Array.from({ length: n }, v6);
  scratch.IA = Array.from({ length: n }, m66);
  scratch.pA = Array.from({ length: n }, v6);
  scratch.a = Array.from({ length: n }, v6);
  scratch.U = new Array(n);   // per body: array of nf 6-vectors (columns of IA*S)
  scratch.D = new Array(n);   // per body: nf x nf, row-major
  scratch.u = new Array(n);   // per body: nf-vector
  scratch.n = n;
}

// model: compiled Model. q: Float64Array(nq). qd, tau: Float64Array(nv).
// fext (optional): per-body spatial force [torque; force] in the body frame.
export function forwardDynamics(model, q, qd, tau, qdd = new Float64Array(model.nv), fext = null) {
  const n = model.bodies.length;
  ensure(n);
  const { Xup, v, c, IA, pA, a, U, D, u } = scratch;
  const { bodies, Xtree, I, S, nf, vIdx, qIdx, qDim } = model;

  const aBase = v6();
  aBase[3] = -model.gravity[0];
  aBase[4] = -model.gravity[1];
  aBase[5] = -model.gravity[2];

  // --- Pass 1: outward. Velocities, bias accelerations, articulated inertias.
  for (let i = 0; i < n; i++) {
    const b = bodies[i], p = b.parent;
    const Si = S[i], nfi = nf[i];
    const qSlice = q.subarray(qIdx[i], qIdx[i] + qDim[i]);

    const { X: XJ } = model.jointTransform(i, qSlice);
    mulMM(XJ, Xtree[i], Xup[i]);

    // vJ = S qd_i  (sum of subspace columns scaled by joint velocities)
    const vJ = v6();
    for (let j = 0; j < nfi; j++) {
      const qdj = qd[vIdx[i] + j];
      for (let k = 0; k < 6; k++) vJ[k] += Si[j][k] * qdj;
    }

    const vi = v[i];
    if (p < 0) vi.set(vJ);
    else { mulMV(Xup[i], v[p], vi); for (let k = 0; k < 6; k++) vi[k] += vJ[k]; }

    crossMotion(vi, vJ, c[i]); // S is constant in its frame, so Sdot = 0

    IA[i].set(I[i]);
    const Iv = mulMV(I[i], vi);
    crossForce(vi, Iv, pA[i]);
    if (fext && fext[i]) for (let k = 0; k < 6; k++) pA[i][k] -= fext[i][k];
  }

  // --- Pass 2: inward. Reduce articulated inertia/bias onto each parent.
  for (let i = n - 1; i >= 0; i--) {
    const b = bodies[i], p = b.parent;
    const Si = S[i], nfi = nf[i];

    if (nfi === 0) { // fixed weld
      if (p >= 0) { congruenceAdd(Xup[i], IA[i], IA[p]); addMtV(Xup[i], pA[i], pA[p]); }
      continue;
    }

    // U_j = IA S_j ;  D = S^T U ;  u = tau - damping*qd - S^T pA
    const Ui = new Array(nfi);
    const Di = new Float64Array(nfi * nfi);
    const ui = new Float64Array(nfi);
    const damped = b.jointType === "revolute" || b.jointType === "prismatic";
    for (let j = 0; j < nfi; j++) {
      Ui[j] = mulMV(IA[i], Si[j]);
      let sp = 0; for (let k = 0; k < 6; k++) sp += Si[j][k] * pA[i][k];
      ui[j] = (tau[vIdx[i] + j] || 0) - sp - (damped ? b.damping * qd[vIdx[i] + j] : 0);
    }
    for (let j = 0; j < nfi; j++)
      for (let l = 0; l < nfi; l++) {
        let s = 0; for (let k = 0; k < 6; k++) s += Si[j][k] * Ui[l][k];
        Di[j * nfi + l] = s;
      }
    U[i] = Ui; D[i] = Di; u[i] = ui;

    if (p >= 0) {
      // Ia = IA - U D^-1 U^T ;  pa = pA + Ia c + U D^-1 u
      const Ut = new Float64Array(nfi * 6);
      for (let j = 0; j < nfi; j++) for (let k = 0; k < 6; k++) Ut[j * 6 + k] = Ui[j][k];
      const M = solve(Di, nfi, Ut, 6);        // D^-1 U^T  (nf x 6)
      const Ia = m66();
      for (let r = 0; r < 6; r++)
        for (let col = 0; col < 6; col++) {
          let s = 0; for (let j = 0; j < nfi; j++) s += Ui[j][r] * M[j * 6 + col];
          Ia[r * 6 + col] = IA[i][r * 6 + col] - s;
        }
      const du = solve(Di, nfi, ui, 1);       // D^-1 u  (nf)
      const pa = v6();
      const Iac = mulMV(Ia, c[i]);
      for (let k = 0; k < 6; k++) {
        let Udu = 0; for (let j = 0; j < nfi; j++) Udu += Ui[j][k] * du[j];
        pa[k] = pA[i][k] + Iac[k] + Udu;
      }
      congruenceAdd(Xup[i], Ia, IA[p]);
      addMtV(Xup[i], pa, pA[p]);
    }
  }

  // --- Pass 3: outward. Spatial accelerations and joint accelerations.
  for (let i = 0; i < n; i++) {
    const b = bodies[i], p = b.parent;
    const Si = S[i], nfi = nf[i];
    const ai = a[i];
    mulMV(Xup[i], p < 0 ? aBase : a[p], ai);
    for (let k = 0; k < 6; k++) ai[k] += c[i][k];

    if (nfi === 0) continue;
    const rhs = new Float64Array(nfi);
    for (let j = 0; j < nfi; j++) {
      let Ua = 0; for (let k = 0; k < 6; k++) Ua += U[i][j][k] * ai[k];
      rhs[j] = u[i][j] - Ua;
    }
    const acc = solve(D[i], nfi, rhs, 1);
    for (let j = 0; j < nfi; j++) {
      qdd[vIdx[i] + j] = acc[j];
      for (let k = 0; k < 6; k++) ai[k] += Si[j][k] * acc[j];
    }
  }

  return qdd;
}

// Solve A X = B for X, A is n x n, B is n x m (row-major). Returns X (n x m).
// Gaussian elimination with partial pivoting. n is small (1 or 6 here).
function solve(A, n, B, m) {
  if (n === 1) {
    const inv = 1 / A[0], X = new Float64Array(m);
    for (let c = 0; c < m; c++) X[c] = B[c] * inv;
    return X;
  }
  const a = Float64Array.from(A), x = Float64Array.from(B);
  for (let col = 0; col < n; col++) {
    let piv = col, best = Math.abs(a[col * n + col]);
    for (let r = col + 1; r < n; r++) {
      const val = Math.abs(a[r * n + col]);
      if (val > best) { best = val; piv = r; }
    }
    if (piv !== col) {
      for (let k = 0; k < n; k++) { const t = a[col * n + k]; a[col * n + k] = a[piv * n + k]; a[piv * n + k] = t; }
      for (let k = 0; k < m; k++) { const t = x[col * m + k]; x[col * m + k] = x[piv * m + k]; x[piv * m + k] = t; }
    }
    const d = a[col * n + col];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = a[r * n + col] / d;
      if (f === 0) continue;
      for (let k = col; k < n; k++) a[r * n + k] -= f * a[col * n + k];
      for (let k = 0; k < m; k++) x[r * m + k] -= f * x[col * m + k];
    }
  }
  for (let r = 0; r < n; r++) {
    const d = a[r * n + r];
    for (let k = 0; k < m; k++) x[r * m + k] /= d;
  }
  return x;
}

function mulMM(A, B, C) {
  for (let i = 0; i < 6; i++)
    for (let j = 0; j < 6; j++) {
      let s = 0; for (let k = 0; k < 6; k++) s += A[i * 6 + k] * B[k * 6 + j];
      C[i * 6 + j] = s;
    }
  return C;
}

function congruenceAdd(Xup, M, target) {
  const t = congruence(Xup, M);
  for (let k = 0; k < 36; k++) target[k] += t[k];
}

function addMtV(Xup, v, target) {
  const t = mulMtV(Xup, v);
  for (let k = 0; k < 6; k++) target[k] += t[k];
}

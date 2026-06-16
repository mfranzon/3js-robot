// Control-oriented dynamics: the quantities you need to write robot controllers.
//
//   inverseDynamics (RNEA) - torque to achieve a desired acceleration; with
//                            qd=0, qdd=0 it gives the gravity (+ bias) torque.
//   massMatrix      (CRBA-equivalent via RNEA columns) - the joint-space inertia.
//   jacobian        - maps joint velocities to an end-effector point's velocity.
//
// All operate on the same reduced-coordinate Model the simulator uses.

import {
  v6, m66, mulMV, mulMtV, mulMM, crossMotion, crossForce,
} from "../physics/spatial.js";

// Recursive Newton-Euler inverse dynamics:  tau = M(q) qdd + C(q,qd) + g(q).
// gravity defaults to the model's; pass [0,0,0] to get pure M qdd + C.
export function inverseDynamics(model, q, qd, qdd, gravity = model.gravity) {
  const n = model.bodies.length;
  const { bodies, Xtree, I, S, nf, vIdx, qIdx, qDim } = model;
  const tau = new Float64Array(model.nv);

  const Xup = Array.from({ length: n }, m66);
  const v = Array.from({ length: n }, v6);
  const a = Array.from({ length: n }, v6);
  const f = Array.from({ length: n }, v6);

  const aBase = v6();
  aBase[3] = -gravity[0]; aBase[4] = -gravity[1]; aBase[5] = -gravity[2];

  // outward: velocities, accelerations, and body forces
  for (let i = 0; i < n; i++) {
    const p = bodies[i].parent, Si = S[i], nfi = nf[i];
    const qSlice = q.subarray(qIdx[i], qIdx[i] + qDim[i]);
    mulMM(model.jointTransform(i, qSlice).X, Xtree[i], Xup[i]);

    const vJ = v6(), aJ = v6();
    for (let j = 0; j < nfi; j++) {
      const qdj = qd[vIdx[i] + j], qddj = qdd[vIdx[i] + j];
      for (let k = 0; k < 6; k++) { vJ[k] += Si[j][k] * qdj; aJ[k] += Si[j][k] * qddj; }
    }

    const vi = v[i], ai = a[i];
    if (p < 0) { vi.set(vJ); mulMV(Xup[i], aBase, ai); }
    else { mulMV(Xup[i], v[p], vi); for (let k = 0; k < 6; k++) vi[k] += vJ[k]; mulMV(Xup[i], a[p], ai); }
    const vxvJ = crossMotion(vi, vJ);
    for (let k = 0; k < 6; k++) ai[k] += aJ[k] + vxvJ[k];

    // f = I a + v x* (I v)
    const Ia = mulMV(I[i], ai), Iv = mulMV(I[i], vi), vxIv = crossForce(vi, Iv);
    for (let k = 0; k < 6; k++) f[i][k] = Ia[k] + vxIv[k];
  }

  // inward: project forces onto joints and accumulate to parents
  for (let i = n - 1; i >= 0; i--) {
    const p = bodies[i].parent, Si = S[i];
    for (let j = 0; j < nf[i]; j++) {
      let s = 0; for (let k = 0; k < 6; k++) s += Si[j][k] * f[i][k];
      tau[vIdx[i] + j] = s;
    }
    if (p >= 0) { const ft = mulMtV(Xup[i], f[i]); for (let k = 0; k < 6; k++) f[p][k] += ft[k]; }
  }
  return tau;
}

// Gravity-compensation torque: hold the configuration against gravity.
export function gravityTorque(model, q) {
  return inverseDynamics(model, q, new Float64Array(model.nv), new Float64Array(model.nv));
}

// Joint-space mass matrix M(q): column j is the torque for unit acceleration of
// DOF j with zero velocity and no gravity.
export function massMatrix(model, q) {
  const nv = model.nv;
  const zero = new Float64Array(nv);
  const M = new Float64Array(nv * nv);
  const e = new Float64Array(nv);
  for (let j = 0; j < nv; j++) {
    e.fill(0); e[j] = 1;
    const col = inverseDynamics(model, q, zero, e, [0, 0, 0]);
    for (let i = 0; i < nv; i++) M[i * nv + j] = col[i];
  }
  return M;
}

// Geometric Jacobian of a point (local to body bi) w.r.t. generalized velocity.
// Returns { Jv, Jw } as row-major 3 x nv. Requires sim.forwardKinematics() first.
// Supports revolute/prismatic chains (free joints are skipped).
export function jacobian(model, sim, bi, localPoint = [0, 0, 0]) {
  const nv = model.nv;
  const Jv = new Float64Array(3 * nv), Jw = new Float64Array(3 * nv);
  const { R, p } = sim.pose[bi];
  // end-effector point in world
  const P = [
    p[0] + R[0] * localPoint[0] + R[1] * localPoint[1] + R[2] * localPoint[2],
    p[1] + R[3] * localPoint[0] + R[4] * localPoint[1] + R[5] * localPoint[2],
    p[2] + R[6] * localPoint[0] + R[7] * localPoint[1] + R[8] * localPoint[2],
  ];

  for (let j = bi; j >= 0; j = model.bodies[j].parent) {
    if (model.nf[j] !== 1) continue;
    const b = model.bodies[j], Rj = sim.pose[j].R, pj = sim.pose[j].p;
    const a = b.axis, n = Math.hypot(a[0], a[1], a[2]) || 1;
    const aw = [
      (Rj[0] * a[0] + Rj[1] * a[1] + Rj[2] * a[2]) / n,
      (Rj[3] * a[0] + Rj[4] * a[1] + Rj[5] * a[2]) / n,
      (Rj[6] * a[0] + Rj[7] * a[1] + Rj[8] * a[2]) / n,
    ];
    const c = model.vIdx[j];
    if (b.jointType === "revolute") {
      const r = [P[0] - pj[0], P[1] - pj[1], P[2] - pj[2]];
      Jv[0 * nv + c] = aw[1] * r[2] - aw[2] * r[1];
      Jv[1 * nv + c] = aw[2] * r[0] - aw[0] * r[2];
      Jv[2 * nv + c] = aw[0] * r[1] - aw[1] * r[0];
      Jw[0 * nv + c] = aw[0]; Jw[1 * nv + c] = aw[1]; Jw[2 * nv + c] = aw[2];
    } else { // prismatic
      Jv[0 * nv + c] = aw[0]; Jv[1 * nv + c] = aw[1]; Jv[2 * nv + c] = aw[2];
    }
  }
  return { Jv, Jw, point: P };
}

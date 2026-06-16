// Position inverse kinematics by damped least squares (Levenberg-Marquardt).
//
//   dq = J^T (J J^T + lambda^2 I)^-1 e,   e = target - end_effector
//
// The damping keeps it well-behaved near singularities and for unreachable
// targets (the arm stretches toward the goal instead of blowing up).

import { jacobian } from "./dynamics.js";

export function ikStep(model, sim, bi, localPoint, target, { lambda = 0.4, maxStep = 0.3 } = {}) {
  sim.forwardKinematics();
  const nv = model.nv;
  const { Jv, point } = jacobian(model, sim, bi, localPoint);
  const e = [target[0] - point[0], target[1] - point[1], target[2] - point[2]];

  // A = Jv Jv^T + lambda^2 I   (3x3)
  const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++) {
      let s = 0; for (let k = 0; k < nv; k++) s += Jv[r * nv + k] * Jv[c * nv + k];
      A[r * 3 + c] = s + (r === c ? lambda * lambda : 0);
    }
  const y = solve3(A, e);

  const dq = new Float64Array(nv);
  let norm = 0;
  for (let k = 0; k < nv; k++) {
    let s = 0; for (let r = 0; r < 3; r++) s += Jv[r * nv + k] * y[r];
    dq[k] = s; norm += s * s;
  }
  norm = Math.sqrt(norm);
  if (norm > maxStep) for (let k = 0; k < nv; k++) dq[k] *= maxStep / norm;

  return { dq, error: Math.hypot(e[0], e[1], e[2]) };
}

// Solve a 3x3 system A x = b (row-major A) by Cramer's rule.
function solve3(A, b) {
  const det =
    A[0] * (A[4] * A[8] - A[5] * A[7]) -
    A[1] * (A[3] * A[8] - A[5] * A[6]) +
    A[2] * (A[3] * A[7] - A[4] * A[6]);
  if (Math.abs(det) < 1e-12) return [0, 0, 0];
  const d = (a0, a1, a2, a3, a4, a5, a6, a7, a8) =>
    a0 * (a4 * a8 - a5 * a7) - a1 * (a3 * a8 - a5 * a6) + a2 * (a3 * a7 - a4 * a6);
  return [
    d(b[0], A[1], A[2], b[1], A[4], A[5], b[2], A[7], A[8]) / det,
    d(A[0], b[0], A[2], A[3], b[1], A[5], A[6], b[2], A[8]) / det,
    d(A[0], A[1], b[0], A[3], A[4], b[1], A[6], A[7], b[2]) / det,
  ];
}

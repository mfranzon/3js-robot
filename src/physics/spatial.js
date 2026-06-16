// Spatial (6D) algebra for Featherstone-style articulated-body dynamics.
//
// Convention: spatial vectors are length-6 arrays ordered [angular(3); linear(3)].
//   motion vector  v = [wx, wy, wz, vx, vy, vz]
//   force  vector  f = [nx, ny, nz, fx, fy, fz]   (torque; force)
//
// Plucker coordinate transforms are stored as 6x6 row-major Float64Array(36).
// A motion vector transforms as  v' = X v.  A force vector as  f' = X^-T f.
// Spatial inertia transforms between frames as  I' = X^T I X  (congruence),
// which is all we need for ABA, so no explicit inverse transform is required.

export const v6 = () => new Float64Array(6);
export const m66 = () => new Float64Array(36);

// 3x3 skew-symmetric matrix of a 3-vector, written into a 6x6 block.
function skew(x, y, z, out, r0, c0) {
  out[(r0 + 0) * 6 + (c0 + 1)] = -z;
  out[(r0 + 0) * 6 + (c0 + 2)] = y;
  out[(r0 + 1) * 6 + (c0 + 0)] = z;
  out[(r0 + 1) * 6 + (c0 + 2)] = -x;
  out[(r0 + 2) * 6 + (c0 + 0)] = -y;
  out[(r0 + 2) * 6 + (c0 + 1)] = x;
}

// Rodrigues rotation matrix R(axis, angle) as a flat 9-array (row-major).
// Rotates vectors in a fixed frame by +angle about a unit axis.
export function rotationMatrix(ax, ay, az, angle) {
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    t * ax * ax + c,      t * ax * ay - s * az, t * ax * az + s * ay,
    t * ax * ay + s * az, t * ay * ay + c,      t * ay * az - s * ax,
    t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c,
  ];
}

// Build a Plucker motion transform from a rotation E (3x3, row-major) and a
// translation r (position of the new frame origin, in old-frame coords):
//   X = [ E            0 ]
//       [ -E*skew(r)   E ]
export function plucker(E, rx, ry, rz) {
  const X = m66();
  // top-left and bottom-right blocks = E
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      const e = E[i * 3 + j];
      X[i * 6 + j] = e;
      X[(i + 3) * 6 + (j + 3)] = e;
    }
  // bottom-left = -E * skew(r)
  // skew(r) columns are r x e_j; compute -E*skew(r) directly.
  const S = [0, -rz, ry, rz, 0, -rx, -ry, rx, 0]; // skew(r), row-major
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      let acc = 0;
      for (let k = 0; k < 3; k++) acc += E[i * 3 + k] * S[k * 3 + j];
      X[(i + 3) * 6 + j] = -acc;
    }
  return X;
}

// y = X v   (6x6 times 6)
export function mulMV(X, v, out = v6()) {
  for (let i = 0; i < 6; i++) {
    let s = 0;
    for (let j = 0; j < 6; j++) s += X[i * 6 + j] * v[j];
    out[i] = s;
  }
  return out;
}

// y = X^T v
export function mulMtV(X, v, out = v6()) {
  for (let i = 0; i < 6; i++) {
    let s = 0;
    for (let j = 0; j < 6; j++) s += X[j * 6 + i] * v[j];
    out[i] = s;
  }
  return out;
}

// C = A B  (6x6)
export function mulMM(A, B, out = m66()) {
  for (let i = 0; i < 6; i++)
    for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let k = 0; k < 6; k++) s += A[i * 6 + k] * B[k * 6 + j];
      out[i * 6 + j] = s;
    }
  return out;
}

// C = A^T B A   (congruence transform, used to move spatial inertia frames)
export function congruence(A, B, out = m66()) {
  const tmp = mulMM(B, A);        // B A
  return mulMtM(A, tmp, out);     // A^T (B A)
}

// C = A^T B
function mulMtM(A, B, out = m66()) {
  for (let i = 0; i < 6; i++)
    for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let k = 0; k < 6; k++) s += A[k * 6 + i] * B[k * 6 + j];
      out[i * 6 + j] = s;
    }
  return out;
}

// Spatial cross product for motion vectors: out = v x m
export function crossMotion(v, m, out = v6()) {
  const wx = v[0], wy = v[1], wz = v[2], vx = v[3], vy = v[4], vz = v[5];
  const ax = m[0], ay = m[1], az = m[2], lx = m[3], ly = m[4], lz = m[5];
  out[0] = wy * az - wz * ay;
  out[1] = wz * ax - wx * az;
  out[2] = wx * ay - wy * ax;
  out[3] = wy * lz - wz * ly + vy * az - vz * ay;
  out[4] = wz * lx - wx * lz + vz * ax - vx * az;
  out[5] = wx * ly - wy * lx + vx * ay - vy * ax;
  return out;
}

// Spatial cross product for force vectors: out = v x* f  =  -(crossMotion)^T f
export function crossForce(v, f, out = v6()) {
  const wx = v[0], wy = v[1], wz = v[2], vx = v[3], vy = v[4], vz = v[5];
  const nx = f[0], ny = f[1], nz = f[2], fx = f[3], fy = f[4], fz = f[5];
  out[0] = wy * nz - wz * ny + vy * fz - vz * fy;
  out[1] = wz * nx - wx * nz + vz * fx - vx * fz;
  out[2] = wx * ny - wy * nx + vx * fy - vy * fx;
  out[3] = wy * fz - wz * fy;
  out[4] = wz * fx - wx * fz;
  out[5] = wx * fy - wy * fx;
  return out;
}

// Spatial rigid-body inertia (6x6) for a body of mass m, center of mass at c
// (in the body frame), and rotational inertia Ic (3x3, about the com).
//   I = [ Ic + m c^x c^xT    m c^x   ]
//       [ m c^xT             m 1     ]
export function rigidBodyInertia(mass, cx, cy, cz, Ic) {
  const I = m66();
  // bottom-right = m * identity
  I[3 * 6 + 3] = mass; I[4 * 6 + 4] = mass; I[5 * 6 + 5] = mass;
  // m * skew(c) in top-right, m * skew(c)^T in bottom-left
  const mc = [0, -cz, cy, cz, 0, -cx, -cy, cx, 0].map((x) => x * mass);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      I[i * 6 + (j + 3)] = mc[i * 3 + j];
      I[(i + 3) * 6 + j] = mc[j * 3 + i];
    }
  // top-left = Ic + m skew(c) skew(c)^T
  // m skew(c) skew(c)^T = m (|c|^2 I - c c^T)
  const c2 = cx * cx + cy * cy + cz * cz;
  const cc = [cx * cx, cx * cy, cx * cz, cx * cy, cy * cy, cy * cz, cx * cz, cy * cz, cz * cz];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      const eye = i === j ? c2 : 0;
      I[i * 6 + j] = Ic[i * 3 + j] + mass * (eye - cc[i * 3 + j]);
    }
  return I;
}

export const skewBlock = skew;

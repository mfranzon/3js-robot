// Minimal quaternion helpers for the free-joint orientation state.
// Quaternions are [x, y, z, w] (matching three.js ordering).

export function quatIdentity() { return [0, 0, 0, 1]; }

// Rotation matrix (row-major, body->world vector rotation) from a unit quat.
export function quatToMatrix(q) {
  const [x, y, z, w] = q;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  return [
    1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
    2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
    2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy),
  ];
}

// Hamilton product a (x) b.
export function quatMul(a, b) {
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

// Quaternion for a rotation of |w|*dt about axis w (body-frame angular vel).
export function quatFromAngularVelocity(wx, wy, wz, dt) {
  const ang = Math.hypot(wx, wy, wz) * dt;
  if (ang < 1e-9) return [0, 0, 0, 1];
  const half = ang / 2, s = Math.sin(half) / (ang / dt);
  return [wx * s, wy * s, wz * s, Math.cos(half)];
}

export function quatFromEuler(r, p, y) {
  const cr = Math.cos(r / 2), sr = Math.sin(r / 2);
  const cp = Math.cos(p / 2), sp = Math.sin(p / 2);
  const cy = Math.cos(y / 2), sy = Math.sin(y / 2);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

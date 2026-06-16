// Collision detection + a soft (compliant) contact model.
//
// Like MuJoCo, contacts are compliant: a penetrating point gets a spring-damper
// normal force plus a regularized Coulomb friction force. Each contact can act
// between two dynamic bodies, or between one body and the static ground plane.
// Forces are applied as external wrenches consumed by the ABA solver.
//
// Shapes: capsule, box, sphere vs the ground; capsule vs box for body-body
// (enough for an articulated arm to push the free cubes around).

// Mass-proportional compliant contact. Stiffness/damping scale with the contact's
// effective mass so the contact's natural frequency (and thus the stable timestep)
// is the same regardless of body mass: a 0.5kg prop and a 65kg robot both rest at
// a small, bounded penetration without exploding or sinking through.
//   kn = KA * m_eff,  bn = BA * m_eff  ->  omega = sqrt(KA), zeta = BA/(2 sqrt(KA))
const KA = 8000;  // contact stiffness per unit mass (1/s^2); rest depth ~ g/KA
const BA = 180;   // contact damping per unit mass (1/s); ~critical at KA
const MU = 0.7;   // Coulomb friction coefficient
const BODY_MAX_DEPTH = 0.02; // cap body-body penetration so deep overlaps can't explode

// --- public entry point ----------------------------------------------------

export function collectContacts(sim) {
  const m = sim.model;
  const raw = [];
  groundContacts(sim, raw);
  bodyBodyContacts(sim, raw);

  const out = [];
  for (const c of raw) {
    const f = resolve(sim, c);
    if (f > 0) out.push({ point: c.point, normal: c.normal, depth: c.depth, force: f });
  }
  return out;
}

// --- detection -------------------------------------------------------------

function groundContacts(sim, raw) {
  const m = sim.model;
  for (let i = 0; i < m.bodies.length; i++) {
    const col = m.bodies[i].collision;
    if (!col) continue;
    const { R, p } = sim.pose[i];
    for (const { local, r } of samplePoints(col)) {
      const c = worldPoint(R, p, local);
      const depth = r - c[1];
      if (depth > 0) raw.push({ a: i, b: -1, point: c, normal: [0, 1, 0], depth });
    }
  }
}

function bodyBodyContacts(sim, raw) {
  const m = sim.model;
  const caps = [], boxes = [];
  for (let i = 0; i < m.bodies.length; i++) {
    const col = m.bodies[i].collision;
    if (col?.type === "capsule") caps.push(i);
    else if (col?.type === "box") boxes.push(i);
  }
  for (const ci of caps)
    for (const bi of boxes) {
      const c = capsuleVsBox(sim, ci, bi);
      if (c) raw.push(c);
    }
}

// Deepest contact between a capsule (body ci) and a box (body bi), or null.
function capsuleVsBox(sim, ci, bi) {
  const cap = sim.model.bodies[ci].collision;
  const A = worldPoint(sim.pose[ci].R, sim.pose[ci].p, cap.a);
  const B = worldPoint(sim.pose[ci].R, sim.pose[ci].p, cap.b);

  let best = null;
  const N = 6;
  for (let s = 0; s <= N; s++) {
    const t = s / N;
    const P = [A[0] + (B[0] - A[0]) * t, A[1] + (B[1] - A[1]) * t, A[2] + (B[2] - A[2]) * t];
    const hit = sphereVsBox(sim, P, cap.radius, bi);
    if (hit && (!best || hit.depth > best.depth)) best = hit;
  }
  if (!best) return null;
  return { a: ci, b: bi, point: best.point, normal: best.normal, depth: best.depth };
}

// Sphere (center, radius) vs box body bi. Returns {point, normal, depth} where
// the normal points from the box toward the sphere center, or null.
function sphereVsBox(sim, center, radius, bi) {
  const box = sim.model.bodies[bi].collision;
  const { R, p } = sim.pose[bi];
  const bc = box.center || [0, 0, 0];
  // sphere center in box-local coords, relative to the box center
  const d = [center[0] - p[0], center[1] - p[1], center[2] - p[2]];
  const lx = R[0] * d[0] + R[3] * d[1] + R[6] * d[2] - bc[0];
  const ly = R[1] * d[0] + R[4] * d[1] + R[7] * d[2] - bc[1];
  const lz = R[2] * d[0] + R[5] * d[1] + R[8] * d[2] - bc[2];
  const [hx, hy, hz] = box.half;
  const cl = [clamp(lx, -hx, hx), clamp(ly, -hy, hy), clamp(lz, -hz, hz)];
  const diff = [lx - cl[0], ly - cl[1], lz - cl[2]];
  let dist = Math.hypot(diff[0], diff[1], diff[2]);

  let nLocal;
  if (dist > 1e-9) {
    nLocal = [diff[0] / dist, diff[1] / dist, diff[2] / dist];
  } else {
    // center inside the box: push out along the least-penetrated axis
    const ex = hx - Math.abs(lx), ey = hy - Math.abs(ly), ez = hz - Math.abs(lz);
    if (ex <= ey && ex <= ez) nLocal = [Math.sign(lx) || 1, 0, 0];
    else if (ey <= ez) nLocal = [0, Math.sign(ly) || 1, 0];
    else nLocal = [0, 0, Math.sign(lz) || 1];
    dist = -Math.min(ex, ey, ez);
  }
  const depth = radius - dist;
  if (depth <= 0) return null;

  // normal back to world
  const n = [
    R[0] * nLocal[0] + R[1] * nLocal[1] + R[2] * nLocal[2],
    R[3] * nLocal[0] + R[4] * nLocal[1] + R[5] * nLocal[2],
    R[6] * nLocal[0] + R[7] * nLocal[1] + R[8] * nLocal[2],
  ];
  // contact point on the box surface (clamp was relative to the box center)
  const surfLocal = [cl[0] + bc[0], cl[1] + bc[1], cl[2] + bc[2]];
  const point = worldPoint(R, p, surfLocal);
  return { point, normal: n, depth };
}

// --- contact response ------------------------------------------------------

// Compute the contact force and apply equal/opposite wrenches. The normal
// points from body b toward body a. Returns the normal force magnitude.
function resolve(sim, c) {
  const { a, b, point, normal, depth } = c;
  const vA = sim.pointVelocity(a, point);
  const vB = b >= 0 ? sim.pointVelocity(b, point) : [0, 0, 0];
  const vrel = [vA[0] - vB[0], vA[1] - vB[1], vA[2] - vB[2]];
  const vn = vrel[0] * normal[0] + vrel[1] * normal[1] + vrel[2] * normal[2];

  // effective mass: the body's own mass on the ground, the reduced mass body-body
  const ma = sim.model.bodies[a].mass;
  const meff = b < 0 ? ma : (ma * sim.model.bodies[b].mass) / (ma + sim.model.bodies[b].mass);
  const d = b < 0 ? depth : Math.min(depth, BODY_MAX_DEPTH);
  let fn = KA * meff * d - BA * meff * vn;
  if (fn < 0) return 0;

  // tangential (friction) opposing slip, capped by the Coulomb cone
  const vt = [vrel[0] - vn * normal[0], vrel[1] - vn * normal[1], vrel[2] - vn * normal[2]];
  const vtMag = Math.hypot(vt[0], vt[1], vt[2]);
  let ft = [0, 0, 0];
  if (vtMag > 1e-9) {
    const ftMag = Math.min(MU * fn, KA * meff * vtMag * 0.05);
    const s = -ftMag / vtMag;
    ft = [vt[0] * s, vt[1] * s, vt[2] * s];
  }

  const F = [fn * normal[0] + ft[0], fn * normal[1] + ft[1], fn * normal[2] + ft[2]];
  addExternalForce(sim, a, point, F);
  if (b >= 0) addExternalForce(sim, b, point, [-F[0], -F[1], -F[2]]);
  return fn;
}

// Accumulate a world force Fw at world point P into body i's body-frame wrench.
function addExternalForce(sim, i, P, Fw) {
  const { R, p } = sim.pose[i];
  const fb = mulT(R, Fw);
  const rb = mulT(R, [P[0] - p[0], P[1] - p[1], P[2] - p[2]]);
  const f = sim.fext[i];
  f[0] += rb[1] * fb[2] - rb[2] * fb[1];
  f[1] += rb[2] * fb[0] - rb[0] * fb[2];
  f[2] += rb[0] * fb[1] - rb[1] * fb[0];
  f[3] += fb[0]; f[4] += fb[1]; f[5] += fb[2];
}

// --- geometry helpers ------------------------------------------------------

function samplePoints(col) {
  if (col.type === "capsule") return [{ local: col.a, r: col.radius }, { local: col.b, r: col.radius }];
  if (col.type === "sphere") return [{ local: col.center || [0, 0, 0], r: col.radius }];
  if (col.type === "cylinder") return cylinderPoints(col);
  if (col.type === "box") {
    const [hx, hy, hz] = col.half;
    const [cx, cy, cz] = col.center || [0, 0, 0];
    const pts = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1])
      pts.push({ local: [cx + sx * hx, cy + sy * hy, cz + sz * hz], r: 0 });
    return pts;
  }
  return [];
}

// A cylinder is sampled as the rim + center of each circular face (points, r=0),
// giving flat ends (no capsule inflation) and, for a wheel lying on its side, a
// rolling contact whose lowest point sweeps as the wheel spins.
function cylinderPoints(col) {
  const { a, b, radius } = col;
  const ax = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const len = Math.hypot(ax[0], ax[1], ax[2]) || 1e-9;
  const d = [ax[0] / len, ax[1] / len, ax[2] / len];
  // two unit vectors perpendicular to the axis
  let t = Math.abs(d[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  let u = cross3(d, t); const un = Math.hypot(u[0], u[1], u[2]); u = [u[0] / un, u[1] / un, u[2] / un];
  const v = cross3(d, u);
  const pts = [{ local: a, r: 0 }, { local: b, r: 0 }];
  const N = 8;
  for (let i = 0; i < N; i++) {
    const c = Math.cos((i / N) * 2 * Math.PI) * radius, s = Math.sin((i / N) * 2 * Math.PI) * radius;
    for (const e of [a, b])
      pts.push({ local: [e[0] + u[0] * c + v[0] * s, e[1] + u[1] * c + v[1] * s, e[2] + u[2] * c + v[2] * s], r: 0 });
  }
  return pts;
}

function cross3(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function worldPoint(R, p, local) {
  return [
    p[0] + R[0] * local[0] + R[1] * local[1] + R[2] * local[2],
    p[1] + R[3] * local[0] + R[4] * local[1] + R[5] * local[2],
    p[2] + R[6] * local[0] + R[7] * local[1] + R[8] * local[2],
  ];
}

function mulT(R, v) {
  return [
    R[0] * v[0] + R[3] * v[1] + R[6] * v[2],
    R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
    R[2] * v[0] + R[5] * v[1] + R[8] * v[2],
  ];
}

function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }

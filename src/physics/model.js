// Model: a kinematic tree of rigid bodies connected by joints.
// This is the reduced-coordinate description the dynamics operates on.
//
// Joints can have different DOF counts, so position state (q) and velocity
// state (qd) have different sizes: a free joint contributes 7 position coords
// (3 translation + 4 quaternion) but 6 velocity DOFs.

import { plucker, rigidBodyInertia, rotationMatrix } from "./spatial.js";
import { quatToMatrix } from "./quat.js";

const IDENTITY3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

// velocity DOFs and position coords per joint type
const NF = { fixed: 0, revolute: 1, prismatic: 1, free: 6 };
const NQ = { fixed: 0, revolute: 1, prismatic: 1, free: 7 };

// Inertia tensors (3x3, about the com, in the body frame) for common shapes.
export const inertia = {
  box(mass, sx, sy, sz) {
    const k = mass / 12;
    return [
      k * (sy * sy + sz * sz), 0, 0,
      0, k * (sx * sx + sz * sz), 0,
      0, 0, k * (sx * sx + sy * sy),
    ];
  },
  cylinder(mass, radius, length) {
    const ir = (mass * radius * radius) / 2;
    const il = (mass * (3 * radius * radius + length * length)) / 12;
    return [il, 0, 0, 0, ir, 0, 0, 0, il];
  },
  sphere(mass, radius) {
    const i = (2 / 5) * mass * radius * radius;
    return [i, 0, 0, 0, i, 0, 0, 0, i];
  },
};

export class Body {
  constructor(opts) {
    Object.assign(this, {
      name: "body",
      parent: -1,
      jointType: "revolute", // "revolute" | "prismatic" | "free" | "fixed"
      axis: [0, 0, 1],
      origin: [0, 0, 0],
      rpy: [0, 0, 0],
      mass: 1,
      com: [0, 0, 0],
      Ic: inertia.sphere(1, 0.1),
      damping: 0.05,
      limit: null,
      collision: null,
      visual: null,
      actuator: null, // { type: "position"|"velocity"|"torque", ...gains }
    }, opts);
  }
}

export class Model {
  constructor() {
    this.bodies = [];
    this.gravity = [0, -9.81, 0];
    this.Xtree = [];   // fixed parent->joint Plucker transform
    this.I = [];       // spatial rigid-body inertia
    this.S = [];       // motion subspace: array of nf columns (each a 6-vector)
    this.nf = [];      // velocity DOFs per body
    this.vIdx = [];    // start index of this body's DOFs in qd/tau
    this.qIdx = [];    // start index of this body's coords in q
    this.qDim = [];    // position-coord count per body (1 or 7)
    this.nv = 0;       // total velocity DOFs
    this.nq = 0;       // total position coords
  }

  add(opts) {
    this.bodies.push(new Body(opts));
    return this.bodies.length - 1;
  }

  compile() {
    this.Xtree = []; this.I = []; this.S = [];
    this.nf = []; this.vIdx = []; this.qIdx = []; this.qDim = [];
    let nv = 0, nq = 0;
    for (const b of this.bodies) {
      const E = rpyToMatrix(b.rpy[0], b.rpy[1], b.rpy[2]);
      this.Xtree.push(plucker(transpose3(E), b.origin[0], b.origin[1], b.origin[2]));
      this.I.push(rigidBodyInertia(b.mass, b.com[0], b.com[1], b.com[2], b.Ic));

      const nf = NF[b.jointType];
      this.S.push(motionSubspace(b));
      this.nf.push(nf);
      this.vIdx.push(nv);
      this.qIdx.push(nq);
      this.qDim.push(NQ[b.jointType]);
      nv += nf;
      nq += NQ[b.jointType];
    }
    this.nv = nv;
    this.nq = nq;
    return this;
  }

  // Joint transform XJ(q) plus the joint's own rotation R and translation t,
  // used both by the dynamics and by forward kinematics. qSlice is the chunk of
  // q belonging to this body (length qDim).
  jointTransform(i, qSlice) {
    const b = this.bodies[i];
    if (b.jointType === "revolute") {
      const [ax, ay, az] = normalize3(b.axis);
      const R = rotationMatrix(ax, ay, az, qSlice[0]);
      return { X: plucker(transpose3(R), 0, 0, 0), R, t: [0, 0, 0] };
    }
    if (b.jointType === "prismatic") {
      const [ax, ay, az] = normalize3(b.axis);
      const t = [ax * qSlice[0], ay * qSlice[0], az * qSlice[0]];
      return { X: plucker(IDENTITY3, t[0], t[1], t[2]), R: IDENTITY3, t };
    }
    if (b.jointType === "free") {
      const t = [qSlice[0], qSlice[1], qSlice[2]];
      const R = quatToMatrix([qSlice[3], qSlice[4], qSlice[5], qSlice[6]]);
      return { X: plucker(transpose3(R), t[0], t[1], t[2]), R, t };
    }
    return { X: plucker(IDENTITY3, 0, 0, 0), R: IDENTITY3, t: [0, 0, 0] };
  }
}

// Motion subspace columns (each a 6-vector [angular; linear]) in the joint frame.
function motionSubspace(b) {
  if (b.jointType === "revolute") {
    const [ax, ay, az] = normalize3(b.axis);
    return [new Float64Array([ax, ay, az, 0, 0, 0])];
  }
  if (b.jointType === "prismatic") {
    const [ax, ay, az] = normalize3(b.axis);
    return [new Float64Array([0, 0, 0, ax, ay, az])];
  }
  if (b.jointType === "free") {
    return [0, 1, 2, 3, 4, 5].map((k) => {
      const s = new Float64Array(6); s[k] = 1; return s;
    });
  }
  return [];
}

export function rpyToMatrix(r, p, y) {
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp,     cp * sr,                cp * cr,
  ];
}

function transpose3(m) {
  return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
}

function normalize3(a) {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / n, a[1] / n, a[2] / n];
}

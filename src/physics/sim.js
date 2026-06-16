// Simulation state + time integration + forward kinematics for rendering.

import { forwardDynamics } from "./aba.js";
import { rpyToMatrix } from "./model.js";
import { collectContacts } from "./contacts.js";
import { quatToMatrix, quatMul, quatNormalize, quatFromAngularVelocity } from "./quat.js";

export class Sim {
  constructor(model) {
    this.model = model.nv !== undefined && model.Xtree.length ? model : model.compile();
    this.nq = this.model.nq;
    this.nv = this.model.nv;
    this.q = new Float64Array(this.nq);
    this.qd = new Float64Array(this.nv);
    this.tau = new Float64Array(this.nv);
    this.qdd = new Float64Array(this.nv);
    this.time = 0;
    this.dt = 1 / 480;   // small enough that stiff corner impacts stay stable
    this.substeps = 8;   // 8 * dt = 1/60s of sim per rendered frame
    this.pose = this.model.bodies.map(() => ({ R: new Float64Array(9), p: new Float64Array(3) }));
    this.vel = this.model.bodies.map(() => ({ w: new Float64Array(3), v: new Float64Array(3) }));
    this.fext = this.model.bodies.map(() => new Float64Array(6));
    this.contacts = [];
    this.controller = null; // optional fn(sim) run every physics step to set tau
    this.resetPositions();
  }

  // Initialize q to identity quaternions for free joints (zero is not a unit quat).
  resetPositions() {
    const m = this.model;
    for (let i = 0; i < m.bodies.length; i++) {
      if (m.bodies[i].jointType === "free") this.q[m.qIdx[i] + 6] = 1; // w = 1
    }
  }

  step() {
    const m = this.model;
    if (this.controller) this.controller(this);
    this.forwardKinematics();
    this.computeVelocities();
    for (const f of this.fext) f.fill(0);
    this.contacts = collectContacts(this);
    forwardDynamics(m, this.q, this.qd, this.tau, this.qdd, this.fext);

    for (let i = 0; i < this.nv; i++) this.qd[i] += this.qdd[i] * this.dt;
    this.integratePositions();
    this.applyLimits();
    this.time += this.dt;
  }

  // Advance q by qd. 1-DOF joints integrate linearly; free joints integrate the
  // body-frame spatial velocity onto position + quaternion.
  integratePositions() {
    const m = this.model, dt = this.dt;
    for (let i = 0; i < m.bodies.length; i++) {
      const b = m.bodies[i], qi = m.qIdx[i], vi = m.vIdx[i];
      if (b.jointType === "revolute" || b.jointType === "prismatic") {
        this.q[qi] += this.qd[vi] * dt;
      } else if (b.jointType === "free") {
        const quat = [this.q[qi + 3], this.q[qi + 4], this.q[qi + 5], this.q[qi + 6]];
        const R = quatToMatrix(quat);
        const wB = [this.qd[vi], this.qd[vi + 1], this.qd[vi + 2]];
        const vB = [this.qd[vi + 3], this.qd[vi + 4], this.qd[vi + 5]];
        // origin moves along R * v_body
        this.q[qi] += (R[0] * vB[0] + R[1] * vB[1] + R[2] * vB[2]) * dt;
        this.q[qi + 1] += (R[3] * vB[0] + R[4] * vB[1] + R[5] * vB[2]) * dt;
        this.q[qi + 2] += (R[6] * vB[0] + R[7] * vB[1] + R[8] * vB[2]) * dt;
        // body-frame angular velocity integrates by right quaternion product
        const dq = quatFromAngularVelocity(wB[0], wB[1], wB[2], dt);
        const nq = quatNormalize(quatMul(quat, dq));
        this.q[qi + 3] = nq[0]; this.q[qi + 4] = nq[1]; this.q[qi + 5] = nq[2]; this.q[qi + 6] = nq[3];
      }
    }
  }

  applyLimits() {
    const m = this.model;
    for (let i = 0; i < m.bodies.length; i++) {
      const b = m.bodies[i];
      if (!b.limit || m.nf[i] !== 1) continue;
      const qi = m.qIdx[i], vi = m.vIdx[i];
      const [lo, hi] = b.limit;
      if (this.q[qi] < lo) { this.q[qi] = lo; if (this.qd[vi] < 0) this.qd[vi] = 0; }
      if (this.q[qi] > hi) { this.q[qi] = hi; if (this.qd[vi] > 0) this.qd[vi] = 0; }
    }
  }

  advance() {
    for (let s = 0; s < this.substeps; s++) this.step();
    this.forwardKinematics();
  }

  // Body-frame world pose per body from current joint positions.
  forwardKinematics() {
    const m = this.model;
    for (let i = 0; i < m.bodies.length; i++) {
      const b = m.bodies[i];
      const qSlice = this.q.subarray(m.qIdx[i], m.qIdx[i] + m.qDim[i]);
      const { R: Rj, t: tj } = m.jointTransform(i, qSlice);

      let Rp, pp;
      if (b.parent < 0) { Rp = IDENTITY9; pp = ZERO3; }
      else { Rp = this.pose[b.parent].R; pp = this.pose[b.parent].p; }

      const Etree = rpyToMatrix(b.rpy[0], b.rpy[1], b.rpy[2]);
      const Rjf = mul3(Rp, Etree);
      const pjf = addv(pp, mul3v(Rp, b.origin));

      const out = this.pose[i];
      out.R.set(mul3(Rjf, Rj));
      out.p.set(addv(pjf, mul3v(Rjf, tj)));
    }
  }

  // World angular velocity and body-origin linear velocity per body.
  computeVelocities() {
    const m = this.model;
    for (let i = 0; i < m.bodies.length; i++) {
      const b = m.bodies[i];
      const { R, p } = this.pose[i];
      const out = this.vel[i];

      let wp, vp, pp;
      if (b.parent < 0) { wp = ZERO3; vp = ZERO3; pp = ZERO3; }
      else { wp = this.vel[b.parent].w; vp = this.vel[b.parent].v; pp = this.pose[b.parent].p; }

      // joint spatial velocity in body frame: vJ = S qd
      const Si = m.S[i], vi = m.vIdx[i];
      let wJx = 0, wJy = 0, wJz = 0, vJx = 0, vJy = 0, vJz = 0;
      for (let j = 0; j < m.nf[i]; j++) {
        const s = Si[j], qd = this.qd[vi + j];
        wJx += s[0] * qd; wJy += s[1] * qd; wJz += s[2] * qd;
        vJx += s[3] * qd; vJy += s[4] * qd; vJz += s[5] * qd;
      }
      const wJ = mul3v(R, [wJx, wJy, wJz]); // to world
      const vJ = mul3v(R, [vJx, vJy, vJz]);
      const r = [p[0] - pp[0], p[1] - pp[1], p[2] - pp[2]];
      const rigid = addv(vp, cross(wp, r));
      out.w.set([wp[0] + wJ[0], wp[1] + wJ[1], wp[2] + wJ[2]]);
      out.v.set([rigid[0] + vJ[0], rigid[1] + vJ[1], rigid[2] + vJ[2]]);
    }
  }

  pointVelocity(i, P) {
    const { v, w } = this.vel[i];
    const { p } = this.pose[i];
    const r = [P[0] - p[0], P[1] - p[1], P[2] - p[2]];
    return addv(v, cross(w, r));
  }
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

const IDENTITY9 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const ZERO3 = [0, 0, 0];

function mul3(a, b) {
  const o = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return o;
}
function mul3v(a, v) {
  return [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
  ];
}
function addv(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }

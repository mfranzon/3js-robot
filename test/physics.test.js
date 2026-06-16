// Headless sanity checks for the dynamics. Run with: node test/physics.test.js
import { makeArm } from "../src/robots/arm.js";
import { Sim } from "../src/physics/sim.js";
import { Model, inertia } from "../src/physics/model.js";
import { loadScene, applyInit } from "../src/scene/loader.js";
import { loadURDF } from "../src/scene/urdf.js";
import { buildActuators, applyActuators } from "../src/control/actuators.js";
import { jacobian, gravityTorque, massMatrix } from "../src/control/dynamics.js";
import { ikStep } from "../src/control/ik.js";
import { readFileSync } from "node:fs";

const playgroundJson = JSON.parse(readFileSync(new URL("../public/scenes/playground.json", import.meta.url)));
const makePlayground = () => loadScene(playgroundJson);

let failures = 0;
const check = (name, ok, detail = "") => {
  console.log(`${ok ? "PASS" : "FAIL"}  ${name}${detail ? "  " + detail : ""}`);
  if (!ok) failures++;
};

// --- 1) Single pendulum: energy stays bounded, it falls, and no NaNs.
{
  const m = new Model();
  const L = 0.6, mass = 1.0;
  m.add({
    parent: -1, jointType: "revolute", axis: [0, 0, 1], origin: [0, 2, 0],
    mass, com: [0, -L / 2, 0], Ic: inertia.cylinder(mass, 0.05, L), damping: 0,
  });
  m.compile();
  const sim = new Sim(m);
  sim.q[0] = Math.PI / 2; // horizontal release
  sim.dt = 1 / 2000;

  const comY = () => 2 + (-L / 2) * Math.cos(sim.q[0]); // height of com
  const y0 = comY();
  let maxY = -Infinity, minY = Infinity, finite = true;
  for (let i = 0; i < 8000; i++) {
    sim.step();
    const y = comY();
    maxY = Math.max(maxY, y); minY = Math.min(minY, y);
    if (!Number.isFinite(sim.q[0])) finite = false;
  }
  check("pendulum stays finite", finite);
  check("pendulum falls below release height", minY < y0 - 0.1, `minY=${minY.toFixed(3)} y0=${y0.toFixed(3)}`);
  check("energy bounded (no gain)", maxY <= y0 + 0.02, `maxY=${maxY.toFixed(4)} y0=${y0.toFixed(4)}`);
  // bottom of swing should reach near straight down (com at 2 - L/2)
  check("reaches near bottom", minY < 2 - L / 2 + 0.02, `minY=${minY.toFixed(3)}`);
}

// --- 2) Pendulum period: small-angle physical-pendulum period check.
{
  const m = new Model();
  const L = 0.6, mass = 1.0;
  m.add({
    parent: -1, jointType: "revolute", axis: [0, 0, 1], origin: [0, 2, 0],
    mass, com: [0, -L / 2, 0], Ic: inertia.cylinder(mass, 0.05, L), damping: 0,
  });
  m.compile();
  const sim = new Sim(m);
  sim.q[0] = 0.05; // small angle
  sim.dt = 1 / 4000;

  // detect zero crossings of q to estimate the period
  let prev = sim.q[0], crossings = [], t = 0;
  for (let i = 0; i < 40000; i++) {
    sim.step(); t += sim.dt;
    if (prev > 0 && sim.q[0] <= 0) crossings.push(t);
    prev = sim.q[0];
  }
  // period = time between same-direction crossings
  const period = crossings.length >= 2 ? crossings[1] - crossings[0] : NaN;
  // physical pendulum: T = 2*pi*sqrt(I_pivot / (m g d)), I_pivot = Icom + m d^2
  const d = L / 2, g = 9.81;
  const Icom = inertia.cylinder(mass, 0.05, L)[8]; // about z
  const Ipivot = Icom + mass * d * d;
  const Texpected = 2 * Math.PI * Math.sqrt(Ipivot / (mass * g * d));
  check("pendulum period matches theory", Math.abs(period - Texpected) < 0.03,
    `T=${period.toFixed(3)}s expected=${Texpected.toFixed(3)}s`);
}

// --- 3) Three-link arm: runs stably without blowing up.
{
  const sim = new Sim(makeArm({ links: 3 }));
  sim.q[0] = Math.PI / 2;
  let finite = true;
  for (let i = 0; i < 5000; i++) {
    sim.step();
    for (let k = 0; k < sim.nq; k++) if (!Number.isFinite(sim.q[k])) finite = false;
  }
  check("3-link arm stays finite", finite, `q=[${Array.from(sim.q).map((x) => x.toFixed(2))}]`);
}

// --- 4) Ground contact: released arm settles on the floor without sinking.
{
  const sim = new Sim(makeArm({ links: 3 }));
  sim.q[0] = Math.PI / 2;
  sim.dt = 1 / 600;
  let maxPen = 0, finite = true;
  const radius = 0.05;
  for (let i = 0; i < 12000; i++) {
    sim.step();
    for (const c of sim.contacts) maxPen = Math.max(maxPen, c.depth);
    for (let k = 0; k < sim.nq; k++) if (!Number.isFinite(sim.q[k])) finite = false;
  }
  // lowest endpoint world y across all bodies at the end
  let lowest = Infinity;
  for (let i = 0; i < sim.model.bodies.length; i++) {
    const { R, p } = sim.pose[i];
    const tip = p[1] + R[3] * 0 + R[4] * -0.6 + R[5] * 0;
    lowest = Math.min(lowest, tip);
  }
  check("contact stays finite", finite);
  check("penetration is bounded", maxPen < 0.03, `maxPen=${maxPen.toFixed(4)}m`);
  check("settles resting on floor", lowest > -radius - 0.03 && lowest < radius + 0.2,
    `lowest tip y=${lowest.toFixed(3)}`);
  const speed = Math.hypot(...Array.from(sim.qd));
  check("comes to rest", speed < 0.5, `|qd|=${speed.toFixed(3)}`);
}

// --- 5) Free-floating body falls at g (no contact).
{
  const m = new Model();
  const s = 0.3, mass = 1;
  m.add({ parent: -1, jointType: "free", mass, com: [0, 0, 0], Ic: inertia.box(mass, s, s, s) });
  m.compile();
  const sim = new Sim(m);
  sim.q[1] = 5; // start high
  sim.dt = 1 / 1000;
  const T = 0.5;
  for (let i = 0; i < T / sim.dt; i++) sim.step();
  const dropExpected = 0.5 * 9.81 * T * T;
  const drop = 5 - sim.q[1];
  const vy = sim.vel[0].v[1];
  check("free body falls under gravity", Math.abs(drop - dropExpected) < 0.02,
    `drop=${drop.toFixed(3)} expected=${dropExpected.toFixed(3)}`);
  check("free body vy matches g*t", Math.abs(vy + 9.81 * T) < 0.05, `vy=${vy.toFixed(3)}`);
}

// --- 6) Free box dropped onto the floor settles flat and at rest.
{
  const m = new Model();
  const s = 0.3, mass = 1;
  m.add({
    parent: -1, jointType: "free", mass, com: [0, 0, 0],
    Ic: inertia.box(mass, s, s, s), collision: { type: "box", half: [s / 2, s / 2, s / 2] },
  });
  m.compile();
  const sim = new Sim(m);
  sim.q[1] = 1.0;
  sim.dt = 1 / 1000;
  let maxPen = 0, finite = true;
  for (let i = 0; i < 4000; i++) {
    sim.step();
    for (const c of sim.contacts) maxPen = Math.max(maxPen, c.depth);
    for (let k = 0; k < sim.nv; k++) if (!Number.isFinite(sim.qd[k])) finite = false;
  }
  const cy = sim.q[1];
  const speed = Math.hypot(...Array.from(sim.qd));
  check("box stays finite", finite);
  check("box penetration bounded", maxPen < 0.03, `maxPen=${maxPen.toFixed(4)}m`);
  check("box rests at half-height", Math.abs(cy - s / 2) < 0.04, `cy=${cy.toFixed(3)} expected=${(s / 2).toFixed(3)}`);
  check("box comes to rest", speed < 0.4, `|qd|=${speed.toFixed(3)}`);
}

// --- 7a) Playground scene stays stable and the cubes settle flat on the floor.
{
  const { model, init } = makePlayground();
  const sim = new Sim(model);
  applyInit(sim, init);
  const cube0 = model.qIdx[3]; // first free box body
  let finite = true;
  for (let f = 0; f < 600; f++) { sim.advance(); for (let k = 0; k < sim.nv; k++) if (!Number.isFinite(sim.qd[k])) finite = false; }
  check("playground stays finite (no blow-up)", finite);
  check("cubes settle flat on the floor", Math.abs(sim.q[cube0 + 1] - 0.125) < 0.05, `box0 y=${sim.q[cube0 + 1].toFixed(3)}`);
}

// --- 7b) Body-body contact: a driven arm link pushes a free box it sweeps into.
{
  const m = new Model();
  const L = 0.6, r = 0.06;
  m.add({
    name: "link", parent: -1, jointType: "revolute", axis: [0, 0, 1], origin: [0, 0.7, 0],
    mass: 1.5, com: [0, -L / 2, 0], Ic: inertia.cylinder(1.5, r, L), damping: 0.02,
    collision: { type: "capsule", a: [0, 0, 0], b: [0, -L, 0], radius: r },
  });
  const s = 0.2, bm = 0.4;
  const bi = m.add({
    name: "box", parent: -1, jointType: "free", mass: bm, com: [0, 0, 0],
    Ic: inertia.box(bm, s, s, s), collision: { type: "box", half: [s / 2, s / 2, s / 2] },
  });
  m.compile();
  const sim = new Sim(m);
  const qi = m.qIdx[bi];
  sim.q[qi] = 0.3; sim.q[qi + 1] = s / 2; // box resting within the link's swing arc
  for (let f = 0; f < 120; f++) sim.advance(); // let the box settle
  const x0 = sim.q[qi];
  sim.controller = (sm) => { sm.tau[0] = 6.0; }; // swing the link into the box
  let finite = true;
  for (let f = 0; f < 400; f++) { sim.advance(); for (let k = 0; k < sim.nv; k++) if (!Number.isFinite(sim.qd[k])) finite = false; }
  check("body-body contact stays finite", finite);
  check("arm link pushes a free box", Math.abs(sim.q[qi] - x0) > 0.1, `box moved=${(sim.q[qi] - x0).toFixed(3)}m`);
}

// --- 8) Every example scene (JSON and URDF) parses and runs without blowing up.
{
  const dir = new URL("../public/scenes/", import.meta.url);
  const manifest = JSON.parse(readFileSync(new URL("manifest.json", dir)));
  for (const s of manifest.scenes) {
    const text = readFileSync(new URL(s.file, dir), "utf8");
    const { model, init } = s.file.endsWith(".urdf") ? loadURDF(text) : loadScene(JSON.parse(text));
    const sim = new Sim(model);
    applyInit(sim, init);
    let finite = true;
    for (let i = 0; i < 1200 && finite; i++) {
      sim.step();
      for (let k = 0; k < sim.nv; k++) if (!Number.isFinite(sim.qd[k])) finite = false;
    }
    check(`scene "${s.file}" runs stably`, finite, `nv=${sim.nv}`);
  }
}

// --- 9) URDF semantics: DOF count, joint types, and gravity actually acts.
{
  const xml = readFileSync(new URL("../public/scenes/arm.urdf", import.meta.url), "utf8");
  const { model } = loadURDF(xml);
  check("URDF builds 3 DOF (fixed base + 3 revolute)", model.nv === 3, `nv=${model.nv}`);
  check("URDF base link is a fixed weld", model.bodies[0].jointType === "fixed");

  const sim = new Sim(model);
  sim.q[1] = 1.0; // perturb the shoulder pitch off vertical
  const y0 = sim.q[1];
  let moved = 0;
  for (let i = 0; i < 2000; i++) { sim.step(); moved = Math.max(moved, Math.abs(sim.q[1] - y0)); }
  check("URDF arm responds to gravity when perturbed", moved > 0.1, `max|dq1|=${moved.toFixed(3)}`);
}

// --- 10) Floating-base URDF: rests on the floor (no fall-through) and rolls.
{
  const xml = readFileSync(new URL("../public/scenes/r2d2.urdf", import.meta.url), "utf8");
  const { model, init } = loadURDF(xml, { floatingBase: true });
  check("floating base adds a free root joint", model.bodies[0].jointType === "free");

  const sim = new Sim(model);
  applyInit(sim, init);
  const bq = model.qIdx[0];
  sim.q[bq + 1] += 0.5; // drop it from a height
  const wheels = [];
  for (let i = 0; i < model.bodies.length; i++) if (/wheel/.test(model.bodies[i].name)) wheels.push(model.vIdx[i]);

  let finite = true;
  for (let f = 0; f < 400; f++) { sim.advance(); for (let k = 0; k < sim.nv; k++) if (!Number.isFinite(sim.qd[k])) finite = false; }
  check("floating robot stays finite after a drop", finite);
  // base should rest at a sensible height, not sink through the floor
  check("robot rests on floor (no fall-through)", sim.q[bq + 1] > 0.2 && sim.q[bq + 1] < 0.7, `base y=${sim.q[bq + 1].toFixed(3)}`);

  const x0 = sim.q[bq], z0 = sim.q[bq + 2];
  sim.controller = (s) => { for (const v of wheels) s.tau[v] = 3.0; };
  for (let f = 0; f < 400; f++) sim.advance();
  const rolled = Math.hypot(sim.q[bq] - x0, sim.q[bq + 2] - z0);
  check("robot rolls when wheels are driven", rolled > 0.1, `rolled=${rolled.toFixed(3)}m`);
}

// --- 11) Actuators: position holds an angle; velocity spins continuously.
{
  // position actuator on a single hinge tracks a target angle
  const mp = new Model();
  mp.add({ parent: -1, jointType: "revolute", axis: [0, 0, 1], origin: [0, 1, 0],
    mass: 1, com: [0, -0.3, 0], Ic: inertia.cylinder(1, 0.05, 0.6), damping: 0.1 });
  mp.compile();
  const sp = new Sim(mp);
  const ap = buildActuators(mp);
  ap[0].setpoint = 1.0; // target 1 rad
  sp.controller = (s) => applyActuators(s, ap);
  for (let i = 0; i < 4000; i++) sp.step();
  // pure PD has a small gravity droop (steady-state offset ~ load/kp)
  check("position actuator holds target angle", Math.abs(sp.q[0] - 1.0) < 0.1, `q=${sp.q[0].toFixed(3)}`);

  // URDF continuous joint becomes a velocity actuator and reaches command speed
  const xml = readFileSync(new URL("../public/scenes/r2d2.urdf", import.meta.url), "utf8");
  const { model, init } = loadURDF(xml, { floatingBase: true });
  const acts = buildActuators(model);
  const wheels = acts.filter((a) => /wheel/.test(a.name));
  check("wheels default to velocity actuators", wheels.length === 4 && wheels.every((a) => a.type === "velocity"));

  const sim = new Sim(model);
  applyInit(sim, init);
  sim.controller = (s) => applyActuators(s, acts);
  for (let f = 0; f < 200; f++) sim.advance();
  for (const a of wheels) a.setpoint = 8;
  for (let f = 0; f < 300; f++) sim.advance();
  const speeds = wheels.map((a) => sim.qd[a.v]);
  const tracking = speeds.every((s) => Math.abs(s - 8) < 2);
  check("velocity actuator spins wheels to command", tracking, `wheel speeds=[${speeds.map((s) => s.toFixed(1))}]`);
}

// --- 12) Control API: Jacobian, gravity compensation, mass matrix, and IK.
{
  const reacher = JSON.parse(readFileSync(new URL("../public/scenes/reacher.json", import.meta.url)));
  const { model } = loadScene(reacher);
  const sim = new Sim(model);
  const ee = model.bodies.length - 1, tip = model.bodies[ee].collision.b;
  sim.q.set([0.5, -0.4, 0.3]);
  sim.forwardKinematics();

  // Jacobian vs finite differences
  const { Jv } = jacobian(model, sim, ee, tip);
  const nv = model.nv, h = 1e-6;
  const eePos = () => {
    sim.forwardKinematics();
    const { R, p } = sim.pose[ee];
    return [p[0] + R[0] * tip[0] + R[1] * tip[1] + R[2] * tip[2],
            p[1] + R[3] * tip[0] + R[4] * tip[1] + R[5] * tip[2],
            p[2] + R[6] * tip[0] + R[7] * tip[1] + R[8] * tip[2]];
  };
  let jErr = 0;
  for (let j = 0; j < nv; j++) {
    const q0 = sim.q[j];
    sim.q[j] = q0 + h; const Pp = eePos();
    sim.q[j] = q0 - h; const Pm = eePos();
    sim.q[j] = q0;
    for (let r = 0; r < 3; r++) jErr = Math.max(jErr, Math.abs((Pp[r] - Pm[r]) / (2 * h) - Jv[r * nv + j]));
  }
  check("Jacobian matches finite differences", jErr < 1e-4, `maxErr=${jErr.toExponential(1)}`);

  // mass matrix symmetry
  const M = massMatrix(model, sim.q);
  let sym = 0;
  for (let i = 0; i < nv; i++) for (let j = 0; j < nv; j++) sym = Math.max(sym, Math.abs(M[i * nv + j] - M[j * nv + i]));
  check("mass matrix is symmetric", sym < 1e-9, `asym=${sym.toExponential(1)}`);

  // gravity compensation holds the arm static
  sim.q.set([0.5, -0.4, 0.3]); sim.qd.fill(0);
  sim.controller = (s) => { const g = gravityTorque(model, s.q); for (let k = 0; k < s.nv; k++) s.tau[k] = g[k]; };
  const q0 = Array.from(sim.q);
  for (let i = 0; i < 2000; i++) sim.step();
  const drift = Math.max(...Array.from(sim.q).map((x, k) => Math.abs(x - q0[k])));
  check("gravity compensation holds arm static", drift < 0.02, `drift=${drift.toFixed(4)}`);

  // IK converges to a reachable target
  sim.q.set([0.4, 0.6, 0.5]);
  const target = [0.9, 1.4, 0];
  for (let it = 0; it < 200; it++) { const { dq } = ikStep(model, sim, ee, tip, target, { lambda: 0.2, maxStep: 0.2 }); for (let k = 0; k < nv; k++) sim.q[k] += dq[k]; }
  const P = eePos();
  check("IK reaches a reachable target", Math.hypot(P[0] - target[0], P[1] - target[1], P[2] - target[2]) < 0.02,
    `err=${Math.hypot(P[0] - target[0], P[1] - target[1], P[2] - target[2]).toFixed(4)}m`);
}

console.log(failures === 0 ? "\nAll checks passed." : `\n${failures} check(s) failed.`);
process.exit(failures === 0 ? 0 : 1);

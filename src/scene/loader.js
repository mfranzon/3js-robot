// Declarative scene loader. Parses a MuJoCo-flavored JSON description into a
// compiled Model. A scene is a tree of bodies under "worldbody"; each body has
// an optional joint connecting it to its parent and a geom that supplies the
// visual, the collision shape, and the inertia all at once.
//
// Example:
// {
//   "option": { "gravity": [0, -9.81, 0] },
//   "worldbody": { "children": [
//     { "name": "link0", "pos": [0,1.5,0],
//       "joint": { "type": "hinge", "axis": [0,0,1], "damping": 0.02 },
//       "geom": { "type": "capsule", "fromto": [0,0,0, 0,-0.6,0], "size": [0.05], "mass": 1, "rgba": "#4fb0ff" },
//       "children": [ ... ] },
//     { "name": "box0", "pos": [0.4,0.6,0], "euler": [0,0.3,0], "freejoint": true,
//       "geom": { "type": "box", "size": [0.15,0.15,0.15], "mass": 0.8, "rgba": "#ff6b9d" } }
//   ] }
// }

import { Model, inertia } from "../physics/model.js";
import { quatFromEuler } from "../physics/quat.js";

const JOINT = { hinge: "revolute", slide: "prismatic", free: "free", fixed: "fixed" };

export function loadScene(json) {
  const model = new Model();
  if (json.option?.gravity) model.gravity = json.option.gravity;
  const init = [];

  const walk = (node, parent) => {
    for (const child of node.children || []) {
      const idx = addBody(model, child, parent, init);
      walk(child, idx);
    }
  };
  walk(json.worldbody || {}, -1);

  model.compile();
  return { model, init };
}

// Apply initial poses (free-body pose, or a single-DOF joint angle) to a sim.
export function applyInit(sim, init) {
  for (const e of init) {
    const qi = sim.model.qIdx[e.body];
    if (e.type === "dof") {
      sim.q[qi] = e.value;
    } else {
      sim.q[qi] = e.pos[0]; sim.q[qi + 1] = e.pos[1]; sim.q[qi + 2] = e.pos[2];
      sim.q[qi + 3] = e.quat[0]; sim.q[qi + 4] = e.quat[1]; sim.q[qi + 5] = e.quat[2]; sim.q[qi + 6] = e.quat[3];
    }
  }
}

function addBody(model, node, parent, init) {
  const free = node.freejoint || node.joint?.type === "free";
  const jt = free ? "free" : node.joint ? JOINT[node.joint.type] : "fixed";
  const g = geomToBody(node.geom || { type: "sphere", size: [0.05], mass: 0.1 });

  const idx = model.add({
    name: node.name || "body",
    parent,
    jointType: jt,
    axis: node.joint?.axis || [0, 0, 1],
    // a free body's pose lives in q, so its tree offset stays at the origin
    origin: free ? [0, 0, 0] : (node.pos || [0, 0, 0]),
    rpy: free ? [0, 0, 0] : (node.euler || [0, 0, 0]),
    damping: node.joint?.damping ?? 0.05,
    limit: node.joint?.range || null,
    actuator: node.joint?.actuator || null,
    mass: g.mass, com: g.com, Ic: g.Ic,
    collision: g.collision, visual: g.visual,
  });

  if (free) {
    const e = node.euler || [0, 0, 0];
    init.push({ type: "free", body: idx, pos: node.pos || [0, 0, 0], quat: node.quat || quatFromEuler(e[0], e[1], e[2]) });
  } else if (node.joint?.init != null) {
    init.push({ type: "dof", body: idx, value: node.joint.init });
  }
  return idx;
}

// Turn a geom spec into mass/com/inertia + collision + visual.
// Conventions follow MuJoCo: box size is half-extents, capsule size is [radius],
// sphere size is [radius]. Mass is given directly, or via density * volume.
function geomToBody(geom) {
  const color = parseColor(geom.rgba);
  const density = geom.density ?? 1000;

  if (geom.type === "box") {
    const [hx, hy, hz] = geom.size;
    const sx = 2 * hx, sy = 2 * hy, sz = 2 * hz;
    const mass = geom.mass ?? density * sx * sy * sz;
    const pos = geom.pos || [0, 0, 0];
    return {
      mass, com: pos, Ic: inertia.box(mass, sx, sy, sz),
      collision: { type: "box", half: [hx, hy, hz] },
      visual: { shape: "box", size: [sx, sy, sz], color, offset: pos },
    };
  }

  if (geom.type === "sphere") {
    const r = geom.size[0];
    const mass = geom.mass ?? density * (4 / 3) * Math.PI * r ** 3;
    const pos = geom.pos || [0, 0, 0];
    return {
      mass, com: pos, Ic: inertia.sphere(mass, r),
      collision: { type: "sphere", center: pos, radius: r },
      visual: { shape: "sphere", size: [r], color, offset: pos },
    };
  }

  if (geom.type === "capsule" || geom.type === "cylinder") {
    const r = geom.size[0];
    // accept either fromto endpoints or pos + half-length along Y
    let a, b;
    if (geom.fromto) {
      a = geom.fromto.slice(0, 3); b = geom.fromto.slice(3, 6);
    } else {
      const h = geom.size[1] ?? 0.5;
      const pos = geom.pos || [0, 0, 0];
      a = [pos[0], pos[1] + h, pos[2]]; b = [pos[0], pos[1] - h, pos[2]];
    }
    const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const len = Math.hypot(d[0], d[1], d[2]) || 1e-6;
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
    const mass = geom.mass ?? density * Math.PI * r * r * len;
    return {
      mass, com: mid, Ic: inertia.cylinder(mass, r, len),
      collision: { type: "capsule", a, b, radius: r },
      visual: { shape: "cylinder", size: [r, len], color, offset: mid, quat: quatAlignY(d) },
    };
  }

  throw new Error(`unknown geom type: ${geom.type}`);
}

// Quaternion [x,y,z,w] rotating the +Y axis onto the (unnormalized) direction d.
function quatAlignY(d) {
  const n = Math.hypot(d[0], d[1], d[2]) || 1;
  const u = [d[0] / n, d[1] / n, d[2] / n];
  const dot = u[1]; // (+Y) . u
  if (dot > 0.99999) return [0, 0, 0, 1];
  if (dot < -0.99999) return [1, 0, 0, 0]; // 180 deg about X
  // axis = Y x u, angle = acos(dot)
  const ax = [u[2], 0, -u[0]]; // (0,1,0) x u
  const al = Math.hypot(ax[0], ax[1], ax[2]) || 1;
  const half = Math.acos(dot) / 2, s = Math.sin(half);
  return [(ax[0] / al) * s, (ax[1] / al) * s, (ax[2] / al) * s, Math.cos(half)];
}

function parseColor(rgba) {
  if (rgba == null) return 0xaaaaaa;
  if (typeof rgba === "number") return rgba;
  if (typeof rgba === "string") return parseInt(rgba.replace("#", ""), 16);
  // [r,g,b] floats 0..1
  return (Math.round(rgba[0] * 255) << 16) | (Math.round(rgba[1] * 255) << 8) | Math.round(rgba[2] * 255);
}

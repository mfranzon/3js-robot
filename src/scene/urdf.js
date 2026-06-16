// URDF import. Parses the standard ROS robot description format (XML) into a
// compiled Model, so real-world robot models can be loaded directly.
//
// URDF separates <link> (rigid bodies, with their own inertial/visual/collision)
// from <joint> (parent->child connections with an origin, axis, and limits).
// We build the kinematic tree by following joints from the root link outward.
//
// Supported: revolute, continuous, prismatic, fixed, floating joints; box,
// cylinder, sphere geometry. Meshes render as a placeholder box (geometry files
// are not loaded). Multiple visuals/collisions per link use the first of each.

import { Model } from "../physics/model.js";
import { Sim } from "../physics/sim.js";
import { applyInit } from "./loader.js";
import { quatFromEuler } from "../physics/quat.js";

const JOINT = {
  revolute: "revolute", continuous: "revolute", prismatic: "prismatic",
  fixed: "fixed", floating: "free",
};

// options.zUp (default true): URDF models are Z-up; our world is Y-up, so the
//   root is rotated -90deg about X to stand the robot up correctly.
// options.floatingBase (default false): give the root link a 6-DOF free joint
//   instead of welding it to the world, so the robot can fall, rest, and roll.
// options.collisionFilter (default none): a regex string; only links whose name
//   matches keep a collision shape. Useful to let a wheeled robot rest on its
//   wheels instead of dragging body/leg geometry (whose capsule approximation
//   would otherwise touch the floor).
export function loadURDF(xmlString, options = {}) {
  const zUp = options.zUp ?? true;
  const floatingBase = options.floatingBase ?? false;
  const collisionFilter = options.collisionFilter ? new RegExp(options.collisionFilter) : null;
  const robot = parseXML(xmlString);
  if (!robot || robot.tag !== "robot") throw new Error("URDF: missing <robot> root");

  // index links and joints
  const links = {};
  for (const l of children(robot, "link")) links[attr(l, "name")] = l;
  const joints = children(robot, "joint").map(parseJoint);
  const materials = {};
  for (const mt of children(robot, "material")) {
    const c = child(mt, "color");
    if (c) materials[attr(mt, "name")] = nums(attr(c, "rgba", "0.7 0.7 0.7 1"));
  }

  // children-of map and root detection (a link that is never a joint child)
  const byParent = {};
  const childLinks = new Set();
  for (const j of joints) {
    (byParent[j.parent] ||= []).push(j);
    childLinks.add(j.child);
  }
  const root = Object.keys(links).find((n) => !childLinks.has(n));
  if (!root) throw new Error("URDF: no root link found");

  const model = new Model();
  const init = [];

  const addLink = (linkName, parentIdx, jointSpec) => {
    const link = links[linkName];
    const isRoot = !jointSpec;
    const type = jointSpec ? JOINT[jointSpec.type] : (floatingBase ? "free" : "fixed");
    if (!type) throw new Error(`URDF: unsupported joint type "${jointSpec.type}"`);
    const free = type === "free";
    const body = linkBody(link, materials);
    if (collisionFilter && !collisionFilter.test(linkName)) body.collision = null;

    const idx = model.add({
      name: linkName,
      parent: parentIdx,
      jointType: type,
      axis: jointSpec?.axis || [0, 0, 1],
      origin: free ? [0, 0, 0] : (jointSpec?.xyz || [0, 0, 0]),
      // reorient a fixed Z-up root into our Y-up world
      rpy: free ? [0, 0, 0] : (jointSpec ? jointSpec.rpy : (zUp ? [-Math.PI / 2, 0, 0] : [0, 0, 0])),
      damping: jointSpec?.damping ?? 0.05,
      limit: jointSpec?.limit || null,
      // continuous joints (e.g. wheels) default to velocity actuators so they
      // can be driven to spin; everything else defaults to position control.
      actuator: jointSpec?.type === "continuous" ? { type: "velocity" } : null,
      mass: body.mass, com: body.com, Ic: body.Ic,
      collision: body.collision, visual: body.visual,
    });

    if (free) {
      // a free root carries the Z-up reorientation in its initial orientation
      const e = isRoot ? (zUp ? [-Math.PI / 2, 0, 0] : [0, 0, 0]) : (jointSpec.rpy || [0, 0, 0]);
      const pos = isRoot ? [0, 0, 0] : (jointSpec.xyz || [0, 0, 0]);
      init.push({ type: "free", body: idx, pos, quat: quatFromEuler(e[0], e[1], e[2]) });
    }
    for (const j of byParent[linkName] || []) addLink(j.child, idx, j);
  };

  addLink(root, -1, null);
  model.compile();

  // A robot's geometry usually extends below its root origin. Lift it so the
  // lowest point rests on y=0 (plus a small gap for a floating base, so it
  // settles onto the floor rather than starting in contact).
  if (options.dropToFloor ?? true) {
    const gap = options.floorGap ?? (floatingBase ? 0.02 : 0);
    const lift = -lowestPoint(model, init) + gap;
    if (Number.isFinite(lift)) {
      const rootInit = init.find((e) => e.body === 0 && e.type === "free");
      if (rootInit) {
        rootInit.pos[1] += lift; // shift the free base's initial height
      } else if (model.bodies[0]?.jointType === "fixed") {
        model.bodies[0].origin[1] += lift;
        model.compile();
      }
    }
  }

  return { model, init };
}

// Lowest world-space y of any collision geometry at the initial pose.
function lowestPoint(model, init) {
  const sim = new Sim(model);
  if (init) applyInit(sim, init);
  sim.forwardKinematics();
  let min = Infinity;
  const wp = (R, p, l) => p[1] + R[3] * l[0] + R[4] * l[1] + R[5] * l[2];
  for (let i = 0; i < model.bodies.length; i++) {
    const { R, p } = sim.pose[i];
    const c = model.bodies[i].collision;
    if (c?.type === "box") {
      const [hx, hy, hz] = c.half, ctr = c.center || [0, 0, 0];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1])
        min = Math.min(min, wp(R, p, [ctr[0] + sx * hx, ctr[1] + sy * hy, ctr[2] + sz * hz]));
    } else if (c?.type === "capsule") {
      min = Math.min(min, wp(R, p, c.a) - c.radius, wp(R, p, c.b) - c.radius);
    } else if (c?.type === "cylinder") {
      // lowest point of a circular face = center_y - radius*sqrt(1 - dy^2),
      // where dy is the y-component of the (world) cylinder axis.
      const axB = [c.b[0] - c.a[0], c.b[1] - c.a[1], c.b[2] - c.a[2]];
      const len = Math.hypot(axB[0], axB[1], axB[2]) || 1;
      const dy = (R[3] * axB[0] + R[4] * axB[1] + R[5] * axB[2]) / len; // world y of axis dir
      const drop = c.radius * Math.sqrt(Math.max(0, 1 - dy * dy));
      min = Math.min(min, wp(R, p, c.a) - drop, wp(R, p, c.b) - drop);
    } else if (c?.type === "sphere") {
      min = Math.min(min, wp(R, p, c.center || [0, 0, 0]) - c.radius);
    }
  }
  return min;
}

function parseJoint(j) {
  const o = child(j, "origin");
  const a = child(j, "axis");
  const lim = child(j, "limit");
  const dyn = child(j, "dynamics");
  return {
    type: attr(j, "type"),
    parent: attr(child(j, "parent"), "link"),
    child: attr(child(j, "child"), "link"),
    xyz: o ? nums(attr(o, "xyz", "0 0 0")) : [0, 0, 0],
    rpy: o ? nums(attr(o, "rpy", "0 0 0")) : [0, 0, 0],
    axis: a ? nums(attr(a, "xyz", "1 0 0")) : [1, 0, 0],
    limit: lim && (attr(lim, "lower") != null || attr(lim, "upper") != null)
      ? [num(attr(lim, "lower", "0")), num(attr(lim, "upper", "0"))] : null,
    damping: dyn ? num(attr(dyn, "damping", "0")) : 0.05,
  };
}

// Inertial (mass/com/inertia) + first visual + first collision -> Body fields.
function linkBody(link, materials) {
  const inertial = child(link, "inertial");
  let mass = 0.1, com = [0, 0, 0], Ic = scaledIdentity(1e-3);
  if (inertial) {
    mass = num(attr(child(inertial, "mass"), "value", "0.1"));
    const io = child(inertial, "origin");
    com = io ? nums(attr(io, "xyz", "0 0 0")) : [0, 0, 0];
    const it = child(inertial, "inertia");
    let I = [
      num(attr(it, "ixx", "0")), num(attr(it, "ixy", "0")), num(attr(it, "ixz", "0")),
      num(attr(it, "ixy", "0")), num(attr(it, "iyy", "0")), num(attr(it, "iyz", "0")),
      num(attr(it, "ixz", "0")), num(attr(it, "iyz", "0")), num(attr(it, "izz", "0")),
    ];
    if (io && attr(io, "rpy")) I = rotateTensor(I, nums(attr(io, "rpy"))); // I_body = R I R^T
    Ic = I;
  }

  const vis = child(link, "visual");
  const col = child(link, "collision");
  return {
    mass, com, Ic,
    visual: vis ? geomVisual(vis, materials) : null,
    collision: (col || vis) ? geomCollision(col || vis) : null,
  };
}

function geomVisual(vis, materials) {
  const g = child(vis, "geometry");
  const o = child(vis, "origin");
  const offset = o ? nums(attr(o, "xyz", "0 0 0")) : [0, 0, 0];
  const rpy = o ? nums(attr(o, "rpy", "0 0 0")) : [0, 0, 0];
  const color = visualColor(vis, materials);

  const box = child(g, "box");
  if (box) { const s = nums(attr(box, "size")); return { shape: "box", size: s, color, offset, rpy }; }
  const cyl = child(g, "cylinder");
  if (cyl) {
    const r = num(attr(cyl, "radius")), l = num(attr(cyl, "length"));
    // URDF cylinders run along local Z; our cylinder mesh is along Y.
    return { shape: "cylinder", size: [r, l], color, offset, quat: quatAlignYtoZ(rpy) };
  }
  const sph = child(g, "sphere");
  if (sph) return { shape: "sphere", size: [num(attr(sph, "radius"))], color, offset, rpy };
  // mesh or unknown: a small placeholder box
  return { shape: "box", size: [0.05, 0.05, 0.05], color, offset, rpy };
}

function geomCollision(col) {
  const g = child(col, "geometry");
  const o = child(col, "origin");
  const offset = o ? nums(attr(o, "xyz", "0 0 0")) : [0, 0, 0];

  const box = child(g, "box");
  if (box) { const s = nums(attr(box, "size")); return { type: "box", half: [s[0] / 2, s[1] / 2, s[2] / 2], center: offset }; }
  const cyl = child(g, "cylinder");
  if (cyl) {
    const r = num(attr(cyl, "radius")), l = num(attr(cyl, "length"));
    // a true cylinder (flat ends) along local Z through the collision origin
    return { type: "cylinder", a: [offset[0], offset[1], offset[2] - l / 2], b: [offset[0], offset[1], offset[2] + l / 2], radius: r };
  }
  const sph = child(g, "sphere");
  if (sph) return { type: "sphere", center: offset, radius: num(attr(sph, "radius")) };
  return null; // mesh collision unsupported
}

function visualColor(vis, materials) {
  const mat = child(vis, "material");
  if (!mat) return 0xb0b6c0;
  const c = child(mat, "color");
  const rgba = c ? nums(attr(c, "rgba")) : materials[attr(mat, "name")];
  if (!rgba) return 0xb0b6c0;
  return (Math.round(rgba[0] * 255) << 16) | (Math.round(rgba[1] * 255) << 8) | Math.round(rgba[2] * 255);
}

// --- tiny 3x3 helpers ---

function rotateTensor(I, rpy) {
  const R = rpyMat(rpy[0], rpy[1], rpy[2]); // body<-inertial
  const RI = mul3(R, I);
  return mul3(RI, transpose3(R));
}
function rpyMat(r, p, y) {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}
function mul3(a, b) {
  const o = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return o;
}
function transpose3(m) { return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]; }
function scaledIdentity(s) { return [s, 0, 0, 0, s, 0, 0, 0, s]; }

// Quaternion combining an rpy with a +90deg X rotation that maps Y onto Z.
function quatAlignYtoZ(rpy) {
  const a = quatFromEuler(rpy[0], rpy[1], rpy[2]);
  const x = [Math.SQRT1_2, 0, 0, Math.SQRT1_2]; // +90deg about X: Y -> Z
  return [
    a[3] * x[0] + a[0] * x[3] + a[1] * x[2] - a[2] * x[1],
    a[3] * x[1] - a[0] * x[2] + a[1] * x[3] + a[2] * x[0],
    a[3] * x[2] + a[0] * x[1] - a[1] * x[0] + a[2] * x[3],
    a[3] * x[3] - a[0] * x[0] - a[1] * x[1] - a[2] * x[2],
  ];
}

// --- minimal XML parser (URDF subset: elements, attributes, nesting) ---

function parseXML(str) {
  str = str.replace(/<!--[\s\S]*?-->/g, "").replace(/<\?[\s\S]*?\?>/g, "");
  const tagRe = /<(\/?)([\w:.\-]+)((?:\s+[\w:.\-]+\s*=\s*"[^"]*")*)\s*(\/?)>/g;
  const rootHolder = { children: [] };
  const stack = [rootHolder];
  let m;
  while ((m = tagRe.exec(str))) {
    const [, close, name, attrStr, selfClose] = m;
    if (close) { stack.pop(); continue; }
    const node = { tag: name, attrs: parseAttrs(attrStr), children: [] };
    stack[stack.length - 1].children.push(node);
    if (!selfClose) stack.push(node);
  }
  return rootHolder.children[0];
}
function parseAttrs(s) {
  const attrs = {};
  const re = /([\w:.\-]+)\s*=\s*"([^"]*)"/g;
  let m;
  while ((m = re.exec(s))) attrs[m[1]] = m[2];
  return attrs;
}
function child(node, tag) { return node?.children.find((c) => c.tag === tag) || null; }
function children(node, tag) { return node ? node.children.filter((c) => c.tag === tag) : []; }
function attr(node, name, dflt = null) { const v = node?.attrs[name]; return v == null ? dflt : v; }
function num(s) { return parseFloat(s); }
function nums(s) { return s.trim().split(/\s+/).map(parseFloat); }

// Embeddable widget: drop a full robotics sim into any container element.
//
//   import { createRoboSim } from ".../robo-sim.js";
//   createRoboSim(document.querySelector("#demo"), { base: "/", scene: "reacher" });
//
// Self-contained: it builds its own canvas + overlays with inline styles, fetches
// scenes from `${base}scenes/`, and runs its own loop. Returns a small handle.

import { Viewer } from "../viewer/viewer.js";
import { Sim } from "../physics/sim.js";
import { loadScene, applyInit } from "../scene/loader.js";
import { loadURDF } from "../scene/urdf.js";
import { buildActuators, applyActuators } from "../control/actuators.js";
import { ikStep } from "../control/ik.js";
import { gravityTorque } from "../control/dynamics.js";

const PANEL = "position:absolute;font:12px/1.4 ui-monospace,Menlo,monospace;color:#c7ccd6;" +
  "background:rgba(20,22,28,0.72);border:1px solid #2a2e38;border-radius:8px;padding:10px 12px;backdrop-filter:blur(6px);";

export function createRoboSim(container, options = {}) {
  const opt = {
    base: "/", scene: null, controls: true, hud: true, picker: true,
    ...options,
  };
  if (getComputedStyle(container).position === "static") container.style.position = "relative";
  container.style.overflow = "hidden";

  const viewer = new Viewer(container);

  const hud = el(container, PANEL + "top:10px;left:10px;pointer-events:none;");
  if (!opt.hud) hud.style.display = "none";
  const panel = el(container, PANEL + "top:10px;right:10px;width:230px;max-height:90%;overflow:auto;");
  if (!opt.controls) panel.style.display = "none";

  const FRAME = 1 / 60, REACH_KP = 80, REACH_KD = 8;
  let sim, model, init, acts = [], controlOn = false;
  let reachMode = false, reachable = false, qTarget = null, eeBody = -1, eeLocal = [0, 0, 0];
  let manifest = { scenes: [] };
  let raf = 0, last = performance.now(), acc = 0, alive = true;

  const fetchJson = async (u) => (await fetch(u)).json();

  async function start() {
    try { manifest = await fetchJson(`${opt.base}scenes/manifest.json`); } catch { manifest = { scenes: [] }; }
    const wanted = opt.scene;
    const startFile = manifest.scenes.find((s) => s.file === wanted || s.file.replace(/\.\w+$/, "") === wanted)?.file
      || manifest.scenes[0]?.file;
    if (startFile) await loadSceneFile(startFile);
    raf = requestAnimationFrame(loop);
  }

  async function loadSceneFile(file) {
    const url = `${opt.base}scenes/${file}`;
    const sopts = manifest.scenes.find((s) => s.file === file)?.options || {};
    const built = file.endsWith(".urdf")
      ? loadURDF(await (await fetch(url)).text(), sopts)
      : loadScene(await (await fetch(url)).json());
    sim = new Sim(built.model);
    model = built.model;
    init = built.init;
    applyInit(sim, init);
    viewer.buildFromModel(model);

    acts = buildActuators(model);
    controlOn = false;
    reachMode = false;
    reachable = model.nf.every((n) => n <= 1) && acts.length >= 2;
    viewer.setTargetVisible(false);
    if (reachable) {
      eeBody = model.bodies.length - 1;
      const col = model.bodies[eeBody].collision;
      eeLocal = col?.type === "capsule" ? col.b : [0, 0, 0];
      qTarget = sim.q.slice();
    }

    sim.controller = (s) => {
      if (reachMode) {
        const g = gravityTorque(model, s.q);
        for (const a of acts) s.tau[a.v] = g[a.v] + REACH_KP * (qTarget[a.q] - s.q[a.q]) - REACH_KD * s.qd[a.v];
      } else if (controlOn) applyActuators(s, acts);
      else s.tau.fill(0);
    };
    buildPanel(file);
  }

  function updateIK() {
    const saved = sim.q.slice();
    sim.q.set(qTarget);
    const { dq } = ikStep(model, sim, eeBody, eeLocal, viewer.getTarget(), { lambda: 0.4, maxStep: 0.12 });
    for (let k = 0; k < model.nv; k++) qTarget[k] += dq[k];
    for (let i = 0; i < model.bodies.length; i++) {
      const b = model.bodies[i];
      if (b.limit && model.nf[i] === 1) {
        const qi = model.qIdx[i];
        qTarget[qi] = Math.max(b.limit[0], Math.min(b.limit[1], qTarget[qi]));
      }
    }
    sim.q.set(saved);
  }

  function currentEE() {
    sim.forwardKinematics();
    const { R, p } = sim.pose[eeBody], l = eeLocal;
    return [
      p[0] + R[0] * l[0] + R[1] * l[1] + R[2] * l[2],
      p[1] + R[3] * l[0] + R[4] * l[1] + R[5] * l[2],
      p[2] + R[6] * l[0] + R[7] * l[1] + R[8] * l[2],
    ];
  }

  // click (not drag) on the canvas sets the reach target
  const canvas = viewer.renderer.domElement;
  let downX = 0, downY = 0;
  canvas.addEventListener("pointerdown", (e) => { downX = e.clientX; downY = e.clientY; });
  canvas.addEventListener("pointerup", (e) => {
    if (!reachMode || Math.hypot(e.clientX - downX, e.clientY - downY) > 6) return;
    const p = viewer.screenToPlane(e.clientX, e.clientY);
    if (p) viewer.setTarget(p[0], p[1], p[2]);
  });

  function loop(now) {
    if (!alive) return;
    raf = requestAnimationFrame(loop);
    let dt = (now - last) / 1000; last = now;
    if (dt > 0.1) dt = 0.1;
    acc += dt;
    if (reachMode) updateIK();
    while (acc >= FRAME) { sim.advance(); acc -= FRAME; }
    viewer.sync(sim);
    viewer.render();
    if (opt.hud) hud.innerHTML =
      `<b style="color:#8fffd1">3js-robot</b><br>` +
      `t=<span style="color:#6cf">${sim.time.toFixed(2)}s</span> ` +
      `dof=<span style="color:#6cf">${sim.nv}</span> ` +
      `contacts=<span style="color:#6cf">${sim.contacts.length}</span><br>` +
      `<span style="color:#33d6ff">&#9679; joint axis</span> ` +
      `<span style="color:#ff5a5a">&#9679; contact</span>`;
  }

  function buildPanel(currentFile) {
    if (!opt.controls) return;
    panel.innerHTML = "";

    if (opt.picker && manifest.scenes.length > 1) {
      const h = head("Scene");
      const sel = document.createElement("select");
      sel.style.cssText = "width:100%;background:#2a2e38;color:#c7ccd6;border:1px solid #3a3f4b;border-radius:6px;padding:4px;margin-bottom:8px;";
      for (const s of manifest.scenes) {
        const o = document.createElement("option");
        o.value = s.file; o.textContent = s.name; if (s.file === currentFile) o.selected = true;
        sel.append(o);
      }
      sel.addEventListener("change", () => loadSceneFile(sel.value));
      panel.append(h, sel);
    }

    panel.append(head(`Actuators (${acts.length} DOF)`));
    const hint = el(panel, "color:#7f8794;font-size:10px;margin:-4px 0 8px;", true);
    hint.textContent = "hover a row to locate its joint";

    acts.forEach((a, k) => {
      const unit = a.type === "position" ? "rad" : a.type === "velocity" ? "rad/s" : "Nm";
      const row = el(panel, "margin:5px 0;", true);
      const cap = el(row, "display:flex;justify-content:space-between;color:#aab0bb;font-size:10px;", true);
      cap.innerHTML = `<span>J${k} ${a.name}</span><span style="color:#6cf">${a.type} (${unit})</span>`;
      const range = document.createElement("input");
      range.type = "range"; range.style.width = "100%";
      range.min = a.range[0]; range.max = a.range[1];
      range.step = (a.range[1] - a.range[0]) / 200; range.value = 0;
      range.addEventListener("input", () => { a.setpoint = parseFloat(range.value); });
      row.addEventListener("mouseenter", () => viewer.highlightJoint(a.body, true));
      row.addEventListener("mouseleave", () => viewer.highlightJoint(a.body, false));
      row.append(range);
    });

    const toggle = button(panel, "Enable actuators", () => {
      controlOn = !controlOn;
      toggle.textContent = controlOn ? "Release (free)" : "Enable actuators";
    });

    if (reachable) {
      const reach = button(panel, "Reach mode: click a target", () => {
        reachMode = !reachMode;
        viewer.setTargetVisible(reachMode);
        if (reachMode) { qTarget = sim.q.slice(); const ee = currentEE(); viewer.setTarget(ee[0], ee[1], ee[2]); }
        reach.textContent = reachMode ? "Reach mode: ON (click to move)" : "Reach mode: click a target";
        reach.style.background = reachMode ? "#2d5a4a" : "#2a2e38";
      });
    }

    button(panel, "Reset", () => {
      sim.q.fill(0); sim.qd.fill(0); sim.time = 0;
      sim.resetPositions(); applyInit(sim, init);
      for (const a of acts) a.setpoint = 0;
      if (reachable) qTarget = sim.q.slice();
    });
  }

  start();

  return {
    get sim() { return sim; },
    get model() { return model; },
    loadScene: loadSceneFile,
    dispose() { alive = false; cancelAnimationFrame(raf); viewer.dispose(); container.innerHTML = ""; },
  };
}

// --- tiny DOM helpers ---
function el(parent, css, plain) {
  const d = document.createElement("div");
  d.style.cssText = (plain ? "" : "") + css;
  parent.append(d);
  return d;
}
function head(text) {
  const h = document.createElement("h3");
  h.textContent = text;
  h.style.cssText = "font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:#6cf;margin:6px 0 8px;";
  return h;
}
function button(parent, text, onclick) {
  const b = document.createElement("button");
  b.textContent = text;
  b.style.cssText = "width:100%;margin-top:8px;padding:6px;cursor:pointer;background:#2a2e38;color:#c7ccd6;border:1px solid #3a3f4b;border-radius:6px;font:12px ui-monospace,monospace;";
  b.addEventListener("click", onclick);
  parent.append(b);
  return b;
}

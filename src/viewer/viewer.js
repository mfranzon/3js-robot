// Three.js viewer: scene, lights, ground, camera controls, and per-body meshes
// driven by the simulation's forward kinematics each frame.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export class Viewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.style.display = "block";
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x15171c);
    this.scene.fog = new THREE.Fog(0x15171c, 8, 22);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 100);
    this.camera.position.set(2.6, 2.2, 4.2);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 1.4, 0);

    this._addLights();
    this._addGround();

    this.bodyMeshes = [];
    this._contactDots = [];
    this._contactGeom = new THREE.SphereGeometry(0.04, 10, 8);
    this._contactMat = new THREE.MeshBasicMaterial({ color: 0xff5a5a });
    this._tmp = new THREE.Matrix4();

    // IK target marker + raycasting onto the reach plane (z = 0 by default)
    this.target = new THREE.Mesh(
      new THREE.SphereGeometry(0.06, 20, 14),
      new THREE.MeshStandardMaterial({ color: 0xff3df0, emissive: 0x661155, emissiveIntensity: 1 })
    );
    this.target.visible = false;
    this.scene.add(this.target);
    this.reachPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    this._ray = new THREE.Raycaster();
    this._ndc = new THREE.Vector2();
    this._q = new THREE.Quaternion();
    this._m3 = new THREE.Matrix4();

    if (typeof ResizeObserver !== "undefined") {
      this._ro = new ResizeObserver(() => this.resize());
      this._ro.observe(container);
    } else addEventListener("resize", () => this.resize());
    this.resize();
  }

  _addLights() {
    this.scene.add(new THREE.HemisphereLight(0x9bb4d0, 0x202428, 0.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(4, 8, 5);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const s = 6;
    Object.assign(key.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 1, far: 30 });
    this.scene.add(key);
  }

  _addGround() {
    const grid = new THREE.GridHelper(20, 40, 0x3a4150, 0x252a33);
    grid.position.y = 0;
    this.scene.add(grid);
    const mat = new THREE.MeshStandardMaterial({ color: 0x1b1e25, roughness: 0.95, metalness: 0 });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), mat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.001;
    floor.receiveShadow = true;
    this.scene.add(floor);
  }

  // Build meshes from a compiled model's visual hints.
  buildFromModel(model) {
    for (const mesh of this.bodyMeshes) this.scene.remove(mesh);
    this.bodyVisuals = [];   // the main geometry mesh per body (for highlighting)
    this.jointArrows = [];   // axis arrow per movable joint (null for others)
    this.jointMarkers = [];  // sphere marker per body

    this.bodyMeshes = model.bodies.map((b, i) => {
      const group = new THREE.Group();
      const visual = b.visual ? makeVisual(b.visual) : null;
      if (visual) group.add(visual);
      this.bodyVisuals[i] = visual;

      // a small joint marker at the body origin (the joint pivot)
      const movable = model.nf[i] === 1;
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(movable ? 0.05 : 0.04, 16, 12),
        new THREE.MeshStandardMaterial({ color: movable ? 0x33d6ff : 0xe8edf4, roughness: 0.4, metalness: 0.3 })
      );
      group.add(marker);
      this.jointMarkers[i] = marker;

      // an arrow along the joint axis so you can see what it moves
      let arrow = null;
      if (movable) {
        const a = b.axis, n = Math.hypot(a[0], a[1], a[2]) || 1;
        const dir = new THREE.Vector3(a[0] / n, a[1] / n, a[2] / n);
        arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), 0.28, 0x33d6ff, 0.08, 0.05);
        group.add(arrow);
      }
      this.jointArrows[i] = arrow;

      group.matrixAutoUpdate = false;
      this.scene.add(group);
      return group;
    });
  }

  // Emphasize one joint (its arrow, marker, and the link it drives).
  highlightJoint(i, on) {
    const arrow = this.jointArrows[i];
    if (arrow) { arrow.setColor(on ? 0xffe14d : 0x33d6ff); arrow.scale.setScalar(on ? 1.7 : 1); }
    const marker = this.jointMarkers[i];
    if (marker) marker.material.color.setHex(on ? 0xffe14d : 0x33d6ff);
    const v = this.bodyVisuals[i];
    if (v?.material) { v.material.emissive.setHex(on ? 0x6b5a10 : 0x000000); }
  }

  // Push the sim's body poses into the meshes.
  sync(sim) {
    for (let i = 0; i < this.bodyMeshes.length; i++) {
      const { R, p } = sim.pose[i];
      const e = this._m3.elements;
      // column-major Matrix4 from row-major 3x3 R and position p
      e[0] = R[0]; e[1] = R[3]; e[2] = R[6]; e[3] = 0;
      e[4] = R[1]; e[5] = R[4]; e[6] = R[7]; e[7] = 0;
      e[8] = R[2]; e[9] = R[5]; e[10] = R[8]; e[11] = 0;
      e[12] = p[0]; e[13] = p[1]; e[14] = p[2]; e[15] = 1;
      this.bodyMeshes[i].matrix.copy(this._m3);
    }
    this._syncContacts(sim.contacts || []);
  }

  // Show a red dot at each active contact point, pooling the meshes.
  _syncContacts(contacts) {
    while (this._contactDots.length < contacts.length) {
      const dot = new THREE.Mesh(this._contactGeom, this._contactMat);
      this.scene.add(dot);
      this._contactDots.push(dot);
    }
    for (let i = 0; i < this._contactDots.length; i++) {
      const dot = this._contactDots[i];
      if (i < contacts.length) {
        const p = contacts[i].point;
        dot.visible = true;
        dot.position.set(p[0], p[1], p[2]);
      } else dot.visible = false;
    }
  }

  setTargetVisible(on) { this.target.visible = on; }
  setTarget(x, y, z) { this.target.position.set(x, y, z); }
  getTarget() { return [this.target.position.x, this.target.position.y, this.target.position.z]; }

  // World point where a screen pixel ray meets the reach plane, or null.
  screenToPlane(clientX, clientY) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this._ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    this._ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    this._ray.setFromCamera(this._ndc, this.camera);
    const hit = new THREE.Vector3();
    return this._ray.ray.intersectPlane(this.reachPlane, hit) ? [hit.x, hit.y, hit.z] : null;
  }

  resize() {
    const w = this.container.clientWidth || innerWidth;
    const h = this.container.clientHeight || innerHeight;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  dispose() {
    this._ro?.disconnect();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

function makeVisual({ shape, size, color, offset = [0, 0, 0], rpy = [0, 0, 0], quat = null }) {
  let geom;
  if (shape === "cylinder") geom = new THREE.CylinderGeometry(size[0], size[0], size[1], 20);
  else if (shape === "box") geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
  else geom = new THREE.SphereGeometry(size[0], 24, 16);
  const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.25 });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.set(offset[0], offset[1], offset[2]);
  if (quat) mesh.quaternion.set(quat[0], quat[1], quat[2], quat[3]);
  else mesh.rotation.set(rpy[0], rpy[1], rpy[2]);
  return mesh;
}

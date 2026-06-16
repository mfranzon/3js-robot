# 3js-robot

A MuJoCo-like robotics simulator built on Three.js, with no external physics engine.
The dynamics are reduced-coordinate (generalized coordinates), the same approach
MuJoCo uses: each joint adds one degree of freedom and forward dynamics is solved
with Featherstone's Articulated-Body Algorithm in 6D spatial coordinates.



https://github.com/user-attachments/assets/00cd0f5e-b1c6-4523-8983-1338a9c66cad



## Run

```bash
npm install
npm run dev          # open the printed localhost URL (also /embed-example.html)
npm run build        # production bundle into dist/
npm run build:embed  # standalone embeddable widget into dist-embed/
npm test             # headless dynamics + control sanity checks
```

## Use as a package

Install from npm (the published package ships the prebuilt widget with Three.js
bundled in, plus the example `scenes/`):

```bash
npm install 3js-robot
```

```js
import { createRoboSim } from "3js-robot";        // registers <robo-sim> too
createRoboSim(document.querySelector("#demo"), { base: "/scenes-host/", scene: "reacher" });
```

Or straight from a CDN, no build step:

```html
<script type="module" src="https://cdn.jsdelivr.net/npm/3js-robot/dist-embed/robo-sim.js"></script>
<robo-sim scene="reacher" base="https://your-host/" style="height:440px"></robo-sim>
```

Copy the `scenes/` folder (from the package or repo) to wherever you point `base`.

## Embedding

The whole sim is packaged as a drop-in widget. Build it with `npm run build:embed`
(outputs `dist-embed/robo-sim.js`, with Three.js bundled in), host the `scenes/`
folder, then:

```html
<script type="module" src="robo-sim.js"></script>
<robo-sim scene="reacher" base="/path-to/" style="height:440px"></robo-sim>
```

Attributes: `scene`, `base` (where `scenes/` lives), and `controls`/`picker`/`hud`
(set to `"false"` to hide). Or call the factory directly:

```js
import { createRoboSim } from "robo-sim.js";
const handle = createRoboSim(el, { base: "/", scene: "pendulum", controls: false });
// handle.sim, handle.loadScene(file), handle.dispose()
```

See `embed-example.html` for live examples (multiple widgets on one page).

## What works today

- Kinematic tree of rigid bodies with revolute, prismatic, and free (6-DOF
  floating base) joints, so fixed-base arms and free bodies run side by side.
- O(n) forward dynamics (ABA), generalized to multi-DOF joints: gravity,
  Coriolis/centrifugal, joint damping.
- Quaternion orientation state for floating bodies, integrated from body-frame
  spatial velocity (so position state nq and velocity state nv differ).
- Symplectic Euler integration with substepping and joint limits.
- Soft (compliant) contact: capsule/box/sphere vs the ground plane, plus
  capsule-vs-box body-body contact, so the arm can push the free cubes around.
  Spring-damper normal force (depth-capped for stability) with regularized
  Coulomb friction, applied as external wrenches through the solver.
- Controller hook that runs at physics rate, so torque tracks state within a
  frame and stays stable.
- Actuators in three modes - position (PD to an angle), velocity (drive to a
  speed, so wheels spin continuously), and torque (direct). Declared per joint;
  URDF continuous joints default to velocity actuators.
- Control API: end-effector Jacobian, inverse dynamics (RNEA, gives gravity
  compensation and Coriolis), and the joint-space mass matrix. Used by a
  click-to-reach IK demo (damped least squares) that drives an arm to a target
  picked in the 3D view, with gravity-compensated PD tracking.
- Mass-proportional compliant contact (stiffness scales with effective mass) so a
  light prop and a heavy robot both rest at a small bounded penetration without
  sinking through or exploding. Cylinders use rim sampling, giving flat ends and
  real rolling contact for wheels.
- Declarative scene format: a MuJoCo-flavored JSON `worldbody` tree with
  `hinge`/`slide`/`free` joints and `geom` shapes that derive the visual,
  collision, and inertia at once. Robots are now data, not code.
- Runtime scene loader: scenes live in `public/scenes/`, are fetched at runtime,
  and are selectable from an in-app picker or via a `?scene=<name>` URL param.
- URDF import: load standard ROS robot descriptions (`.urdf`) directly. Links,
  joints, inertials, limits, and box/cylinder/sphere geometry are mapped onto the
  Model; Z-up models are reoriented into the Y-up world automatically.
- PD position actuators driven from on-screen sliders.
- Three.js viewer: orbit camera, shadows, grid, per-body meshes, contact markers.
- Verified: single-pendulum period matches the physical-pendulum formula; energy
  stays bounded with zero damping; multi-link chains stay stable; a released arm
  settles on the floor with penetration bounded to ~1.4cm and comes to rest.

## Layout

```
src/physics/spatial.js   6D spatial algebra (Plucker transforms, cross products, inertia)
src/physics/model.js     Body/Model: kinematic tree, shape inertias, compile step
src/physics/aba.js       Featherstone articulated-body forward dynamics
src/physics/quat.js      Quaternion helpers for the floating-base orientation
src/physics/sim.js       State, time integration, forward kinematics for rendering
src/control/actuators.js Position / velocity / torque joint actuators
src/control/dynamics.js  Jacobian, inverse dynamics (RNEA), mass matrix
src/control/ik.js        Damped-least-squares inverse kinematics
src/scene/loader.js      Declarative JSON -> compiled Model loader
src/scene/urdf.js        URDF (XML) -> compiled Model loader
public/scenes/*          Runtime-loadable example scenes (JSON + URDF) + manifest
src/robots/arm.js        Sample N-link planar arm (used by the tests)
src/viewer/viewer.js     Three.js scene + mesh sync (sized to its container)
src/embed/robosim.js     createRoboSim() - the self-contained embeddable widget
src/embed/index.js       <robo-sim> web component + embed entry point
src/main.js              Mounts the widget full-page
```

## Scene format

```jsonc
{
  "option": { "gravity": [0, -9.81, 0] },
  "worldbody": { "children": [
    { "name": "link0", "pos": [-0.5, 1.5, 0],
      "joint": { "type": "hinge", "axis": [0,0,1], "damping": 0.02 },
      "geom": { "type": "capsule", "fromto": [0,0,0, 0,-0.6,0], "size": [0.05], "mass": 1, "rgba": "#4fb0ff" },
      "children": [ /* nested child bodies */ ] },
    { "name": "box0", "pos": [0, 0.6, 0], "euler": [0, 0.3, 0], "freejoint": true,
      "geom": { "type": "box", "size": [0.125,0.125,0.125], "mass": 0.8, "rgba": "#ff6b9d" } }
  ] }
}
```

Joints: `hinge` (revolute), `slide` (prismatic), `free` (6-DOF), or omit for a
fixed weld. A `joint.init` value sets the joint's starting angle, and
`joint.actuator` (`{ "type": "velocity", "kv": 3 }`, or `position`/`torque`)
sets how its slider drives it. Geoms: `box`
(half-extents), `sphere`, `capsule`/`cylinder` (`fromto` + radius). Mass is given
directly or as `density` * volume.

### Adding / picking scenes

Drop a `.json` file in `public/scenes/` and add an entry to
`public/scenes/manifest.json`. It then appears in the in-app picker and can be
opened directly with `?scene=<filename-without-.json>`. The bundled examples,
in increasing complexity:

- `pendulum.json`   - 2-DOF chaotic double pendulum.
- `playground.json` - 3-DOF arm + free cubes, body-body contact.
- `reacher.json`    - 3-DOF arm for the click-to-reach IK demo.
- `branched.json`   - 5-DOF branched kinematic tree swatting mixed props.
- `arm.urdf`        - 3-DOF arm loaded from a standard URDF file.

URDF support covers links/joints/inertials/limits and box/cylinder/sphere
geometry. Mesh geometry renders as a placeholder (mesh files are not loaded), and
the first visual/collision per link is used.

## Roadmap (toward MuJoCo parity)

- More body-body collision pairs (box/box, capsule/capsule); currently only
  capsule/box plus ground contact.
- Multiple geoms per body, and loading URDF mesh geometry (STL/OBJ/DAE).
- Spherical joints and fixed-body welding.
- Inverse dynamics (RNEA) and the mass matrix (CRBA) for control.
- An MJCF-like scene description (XML/JSON) to load robots declaratively.
- Velocity/torque actuators and sensors.

## Contributing

Issues and PRs welcome. To work on it locally: `npm install`, then `npm run dev`
for the app or `npm test` for the headless dynamics/control checks. Keep changes
small and covered by a check in `test/physics.test.js` where it makes sense.

## License

MIT - see [LICENSE](LICENSE).

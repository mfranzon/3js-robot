// Actuators: turn a per-joint setpoint into a joint torque each control step.
//
// Three modes, all ultimately producing a torque the dynamics integrates:
//   position - PD to a target angle:    tau = kp (x* - q) - kd qd
//   velocity - P to a target velocity:  tau = kv (v* - qd)
//   torque   - the setpoint is the torque directly
// Each actuator clamps its output to +/- forceLimit.

const DEFAULTS = {
  position: { kp: 40, kd: 4, range: [-Math.PI, Math.PI], forceLimit: 200 },
  velocity: { kv: 3, range: [-12, 12], forceLimit: 200 },
  torque: { range: [-40, 40], forceLimit: Infinity },
};

// Build a normalized actuator descriptor per single-DOF joint of the model.
export function buildActuators(model) {
  const acts = [];
  for (let i = 0; i < model.bodies.length; i++) {
    if (model.nf[i] !== 1) continue;
    const b = model.bodies[i];
    const spec = b.actuator || { type: "position" };
    const d = DEFAULTS[spec.type] || DEFAULTS.position;
    const range = spec.range || (spec.type === "position" && b.limit) || d.range;
    acts.push({
      name: b.name,
      body: i,
      type: spec.type,
      q: model.qIdx[i],
      v: model.vIdx[i],
      kp: spec.kp ?? d.kp,
      kd: spec.kd ?? d.kd,
      kv: spec.kv ?? d.kv,
      range,
      forceLimit: spec.forceLimit ?? d.forceLimit,
      setpoint: 0,
    });
  }
  return acts;
}

// Write torques into sim.tau from the actuators' current setpoints.
export function applyActuators(sim, acts) {
  for (const a of acts) {
    let tau;
    if (a.type === "velocity") tau = a.kv * (a.setpoint - sim.qd[a.v]);
    else if (a.type === "torque") tau = a.setpoint;
    else tau = a.kp * (a.setpoint - sim.q[a.q]) - a.kd * sim.qd[a.v];
    sim.tau[a.v] = clamp(tau, -a.forceLimit, a.forceLimit);
  }
}

const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

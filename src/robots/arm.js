// A simple N-link planar arm: revolute joints about Z, links hanging along -Y.
// With several links and no control torque it behaves as a chaotic pendulum,
// which is a good visual test that the articulated dynamics are correct.

import { Model, inertia } from "../physics/model.js";

export function makeArm({ links = 3, length = 0.6, radius = 0.05, mass = 1.0, baseHeight = 1.5 } = {}) {
  const m = new Model();
  const colors = [0x4fb0ff, 0x6ee7a8, 0xffd166, 0xff6b9d, 0xc792ea];

  for (let i = 0; i < links; i++) {
    const origin = i === 0 ? [0, baseHeight, 0] : [0, -length, 0];
    m.add({
      name: `link${i}`,
      parent: i - 1,
      jointType: "revolute",
      axis: [0, 0, 1],
      origin,
      mass,
      com: [0, -length / 2, 0],
      Ic: inertia.cylinder(mass, radius, length),
      damping: 0.02,
      collision: { type: "capsule", a: [0, 0, 0], b: [0, -length, 0], radius },
      visual: {
        shape: "cylinder",
        size: [radius, length],
        color: colors[i % colors.length],
        offset: [0, -length / 2, 0], // centered on the link
      },
    });
  }
  return m.compile();
}

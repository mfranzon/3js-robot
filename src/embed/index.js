// Embed entry point: exposes the factory and registers a <robo-sim> element.
//
//   <robo-sim scene="reacher" style="height:480px"></robo-sim>
//   <robo-sim scene="pendulum" controls="false" base="/assets/"></robo-sim>
//
// Attributes: scene, base, controls, picker, hud (booleans default true, set
// "false" to disable). The element needs a height (via CSS or style).

import { createRoboSim } from "./robosim.js";

export { createRoboSim };

class RoboSimElement extends HTMLElement {
  connectedCallback() {
    if (this._handle) return;
    if (getComputedStyle(this).display === "inline") this.style.display = "block";
    const mount = document.createElement("div");
    mount.style.cssText = "width:100%;height:100%;";
    this.appendChild(mount);
    if (this.offsetHeight === 0) this.style.height = "420px"; // sensible default

    const bool = (name, def) => (this.hasAttribute(name) ? this.getAttribute(name) !== "false" : def);
    this._handle = createRoboSim(mount, {
      base: this.getAttribute("base") || "/",
      scene: this.getAttribute("scene") || null,
      controls: bool("controls", true),
      picker: bool("picker", true),
      hud: bool("hud", true),
    });
  }

  disconnectedCallback() {
    this._handle?.dispose();
    this._handle = null;
  }
}

if (typeof customElements !== "undefined" && !customElements.get("robo-sim")) {
  customElements.define("robo-sim", RoboSimElement);
}

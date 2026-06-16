import { createRoboSim } from "./embed/index.js";

// The full-page app is just the embeddable widget mounted on #app.
const params = new URLSearchParams(location.search);
createRoboSim(document.getElementById("app"), {
  base: import.meta.env.BASE_URL,
  scene: params.get("scene"),
});

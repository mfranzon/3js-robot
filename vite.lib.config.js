// Build the embeddable widget as a standalone library (Three.js bundled in), so
// any site can include one file and use <robo-sim> or createRoboSim().
//   npm run build:embed   ->   dist-embed/robo-sim.js (ES) + robo-sim.umd.cjs
import { defineConfig } from "vite";

export default defineConfig({
  build: {
    outDir: "dist-embed",
    emptyOutDir: true,
    lib: {
      entry: "src/embed/index.js",
      name: "RoboSim",
      fileName: (format) => (format === "es" ? "robo-sim.js" : "robo-sim.umd.cjs"),
      formats: ["es", "umd"],
    },
  },
});

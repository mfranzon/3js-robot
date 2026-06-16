// App build. Two HTML entries: the full-page app and the embedding examples page.
import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        main: fileURLToPath(new URL("index.html", import.meta.url)),
        embed: fileURLToPath(new URL("embed-example.html", import.meta.url)),
      },
    },
  },
});

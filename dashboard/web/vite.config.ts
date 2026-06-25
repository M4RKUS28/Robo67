import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Backend bridge (mock or live). Override with VITE/BACKEND env if needed.
const BACKEND = process.env.ROBO67_BACKEND ?? "http://127.0.0.1:8088";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    strictPort: false,
    proxy: {
      // Same-origin in dev: the frontend talks to /api, Vite proxies to the
      // Python bridge. SSE + MJPEG need buffering disabled (ws:false is fine,
      // these are plain HTTP streams).
      "/api": {
        target: BACKEND,
        changeOrigin: true,
      },
    },
  },
  build: {
    outDir: "dist",
    sourcemap: false,
  },
});

import { defineConfig, loadEnv } from "vite";

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), "");
  const target = env.BCR_BROWSER_BRIDGE_URL || "http://127.0.0.1:8080";

  return {
    server: {
      host: env.BCR_UI_HOST || "127.0.0.1",
      port: Number(env.BCR_UI_PORT || 5173),
      proxy: {
        "/api": {
          target,
          changeOrigin: true,
        },
      },
    },
    preview: {
      host: env.BCR_UI_HOST || "127.0.0.1",
      port: Number(env.BCR_UI_PREVIEW_PORT || 4173),
    },
  };
});

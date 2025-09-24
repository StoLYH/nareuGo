import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
// import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    // vueDevTools(), // 잠시 비활성화하여 변환 주입 간섭 제거
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url))
    },
  },
  server: {
    host: true,
    port: 5173,
    strictPort: true,
    hmr: {
      port: 5173,
    },
    // Windows/네트워크 파일시스템에서 변경 감지 안정화를 위해 폴링 워처 사용
    watch: {
      usePolling: true,
      interval: 100,
    },
  },
  define: {
    global: 'globalThis',
  },
})

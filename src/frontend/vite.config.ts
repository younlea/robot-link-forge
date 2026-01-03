import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    host: true, // Listen on all addresses
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
      },
      '/static': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
      },
    },
    hmr: {
      clientPort: 5173,
    },
  },
  optimizeDeps: {
    esbuildOptions: {
      // Disable source maps for dependencies to avoid "No sources declared" warnings
      sourcemap: false as boolean | 'inline' | 'external' | 'both',
    },
  },
  build: {
    sourcemap: false,
  }
})
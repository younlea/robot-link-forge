import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    host: true, // This will expose the server to the network
    allowedHosts: ['*'],
  },
  optimizeDeps: {
    esbuildOptions: {
        # Disable source maps for dependencies to avoid "No sources declared" warnings
        sourcemap: false,
    },
  },
  build: {
    sourcemap: false,
  }
})
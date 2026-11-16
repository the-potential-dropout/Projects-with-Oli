import { defineConfig } from 'vite';

export default defineConfig({
  // Relative paths so the built app works from any folder (GitHub Pages, a USB stick...)
  base: './',
  build: {
    // Safari 15+ (iPhone/iPad), Samsung Internet, Chrome, Edge, Firefox
    target: ['es2020', 'safari15', 'chrome90', 'firefox90', 'edge90'],
    chunkSizeWarningLimit: 900,
  },
  server: { host: true },
});

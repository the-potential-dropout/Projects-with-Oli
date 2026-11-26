import { defineConfig } from 'vite';
import { readFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

// The compiled engineering design notebook (docs/notebook/build_notebook.py) is
// published next to the simulator as notebook.html, with a link back to the sim.
const NOTEBOOK = fileURLToPath(new URL('./docs/notebook/Hexapod_Mk1_Design_Notebook.html', import.meta.url));
const BACK_LINK =
  '<p style="margin:0 0 1rem"><a href="./" style="color:#c44f2a;font-weight:600;text-decoration:none">← Back to the simulator</a></p>';

function notebookHtml() {
  if (!existsSync(NOTEBOOK)) return null;
  return readFileSync(NOTEBOOK, 'utf8').replace('<body>', '<body>' + BACK_LINK);
}

function designNotebook() {
  return {
    name: 'design-notebook',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (!req.url || !req.url.split('?')[0].endsWith('/notebook.html')) return next();
        const html = notebookHtml();
        if (!html) return next();
        res.setHeader('Content-Type', 'text/html; charset=utf-8');
        res.end(html);
      });
    },
    generateBundle() {
      const html = notebookHtml();
      if (!html) {
        this.warn('design notebook not compiled: run python docs/notebook/build_notebook.py');
        return;
      }
      this.emitFile({ type: 'asset', fileName: 'notebook.html', source: html });
    },
  };
}

export default defineConfig({
  // Relative paths so the built app works from any folder (GitHub Pages, a USB stick...)
  base: './',
  plugins: [designNotebook()],
  build: {
    // Safari 15+ (iPhone/iPad), Samsung Internet, Chrome, Edge, Firefox
    target: ['es2020', 'safari15', 'chrome90', 'firefox90', 'edge90'],
    chunkSizeWarningLimit: 900,
  },
  server: { host: true },
});

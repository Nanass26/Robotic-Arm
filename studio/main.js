// ORION Studio — point d’entrée.
import { App } from './ui/app.js';

const root = document.getElementById('app');
try {
  window.orion = new App(root);
} catch (e) {
  console.error(e);
  root.innerHTML = `<div style="padding:24px;font:14px system-ui">Erreur au démarrage d’ORION Studio : ${String(e.message || e)}</div>`;
}

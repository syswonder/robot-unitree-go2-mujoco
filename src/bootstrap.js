import { createControlPanel } from './ControlPanel.js';

const query = new URLSearchParams(window.location.search);
// Ordinary visits only inspect the bridge. The launcher explicitly owns physics with runtime=1.
const ownsPhysics = query.get('backend') !== 'native' && (query.get('runtime') === '1' || query.get('bridge') === 'off');
const backend = ownsPhysics ? 'web' : 'native';
document.body.dataset.backend = backend;
if (!ownsPhysics) {
  const native = createControlPanel('native');
  window.mujocoApp = Object.freeze({
    getState: () => ({ backend: 'native', ...(native.state?.state ?? native.state ?? {}) })
  });
} else {
  document.querySelector('#control-panel')?.remove();
  try {
    await import('./main.js');
  } catch (error) {
    const detail = `Web startup failed: ${error.message ?? error}`;
    window.dispatchEvent(new CustomEvent('mujoco-error', { detail }));
    const message = document.createElement('pre');
    message.id = 'startup-error';
    message.textContent = detail;
    document.body.appendChild(message);
    console.error(error);
  }
}

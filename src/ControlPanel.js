/** HTTP control transport for the native viewer's existing bridge, never a second physics client. */
export class NativeBridgeClient {
  /** Resolve the shared bridge HTTP address, allowing remote hosts and explicit overrides. */
  constructor(location = window.location, fetcher = fetch) {
    const query = new URLSearchParams(location.search);
    this.url = (query.get('bridgeHttpUrl') || `${location.protocol}//${location.hostname || '127.0.0.1'}:8766`).replace(/\/$/, '');
    this.fetcher = fetcher.bind(globalThis);
    this.health = null;
    this.state = null;
  }

  /** Reject HTTP, JSON, and negative command acknowledgements with their server diagnostics. */
  async request(path, options = {}) {
    const response = await this.fetcher(`${this.url}${path}`, {
      cache: 'no-store', signal: AbortSignal.timeout(path === '/command' ? 120000 : 3000), ...options
    });
    const payload = await response.json();
    if (!response.ok || payload.accepted === false || payload.ok === false) {
      throw new Error(payload.error || payload.message || `Bridge HTTP ${response.status}`);
    }
    return payload;
  }

  /** Wait for the runtime acknowledgement before displaying a successful command. */
  command(command) {
    return this.request('/command', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(command)
    });
  }

  /** Refresh independent bridge health and the last runtime state for the control panel. */
  async refresh() {
    this.health = await this.request('/health');
    this.state = await this.request('/state');
    return this.state;
  }
}

/** Mount explicit policy controls shared by the web physics and native control-only modes. */
export function createControlPanel(backend) {
  const panel = document.querySelector('#control-panel');
  const status = panel.querySelector('#policy-status');
  const error = panel.querySelector('#policy-error');
  const connection = panel.querySelector('#bridge-status');
  const diagnostics = panel.querySelector('#bridge-diagnostics');
  const load = panel.querySelector('#policy-load');
  const unload = panel.querySelector('#policy-unload');
  const native = backend === 'native' ? new NativeBridgeClient() : null;
  panel.querySelector('#backend-status').textContent = backend === 'native' ? 'Native viewer' : 'Web WASM';
  let pending = false;
  let commandError = '';
  let startupError = '';
  let timer;

  /** Update visible status without hiding load failures when the policy is unloaded. */
  const refresh = async () => {
    try {
      if (native) await native.refresh();
      const state = native ? (native.state?.state ?? native.state) : window.mujocoApp?.getState();
      const health = native?.health ?? window.mujocoBridge?.status();
      const policy = state?.policy;
      const connected = health?.runtimeConnected ?? health?.connected ?? false;
      if (native) panel.querySelector('#backend-status').textContent = `${health?.backend === 'web' ? 'Web' : 'Native'} control panel`;
      status.textContent = pending ? 'Loading MoE Rough...' : policy?.loaded ? 'MoE Rough loaded' : 'ONNX unloaded - scripted gait';
      connection.textContent = connected ? 'Bridge connected' : 'Bridge disconnected';
      error.textContent = startupError || commandError || policy?.error || health?.lastError || '';
      load.disabled = pending || Boolean(policy?.loaded) || !state || (native && !connected);
      unload.disabled = !pending && !policy?.loaded;
      diagnostics.textContent = JSON.stringify({ health: health ?? null, state: state ?? null }, null, 2);
    } catch (failure) {
      connection.textContent = 'Bridge unavailable';
      error.textContent = startupError || commandError || String(failure.message ?? failure);
      load.disabled = true;
    }
  };

  /** Execute only explicit button actions, with unload also able to cancel a pending web load. */
  const command = async (action) => {
    pending = action === 'load';
    commandError = '';
    load.disabled = true;
    unload.disabled = false;
    status.textContent = action === 'load' ? 'Loading MoE Rough...' : 'Unloading MoE Rough...';
    try {
      const message = { type: 'policy', action, id: 'moe_rough' };
      if (native) await native.command(message);
      else await window.mujocoBridge.policyCommand(message);
    } catch (failure) {
      commandError = String(failure.message ?? failure);
    } finally {
      pending = false;
      await refresh();
    }
  };
  load.addEventListener('click', () => command('load'));
  unload.addEventListener('click', () => command('unload'));
  window.addEventListener('mujoco-error', (event) => { startupError = event.detail; error.textContent = startupError; });

  /** Poll serially so a slow native bridge cannot accumulate overlapping requests. */
  const poll = async () => {
    await refresh();
    timer = setTimeout(poll, 1000);
  };
  poll();
  window.addEventListener('pagehide', () => clearTimeout(timer), { once: true });
  return native;
}

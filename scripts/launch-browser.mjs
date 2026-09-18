import { chromium } from 'playwright';

const visible = process.env.SIM_HEADLESS !== '1';
const url = process.env.SIM_URL || 'http://127.0.0.1:5181/?runtime=1';
const debugPort = process.env.SIM_BROWSER_DEBUG_PORT;
if (debugPort && (!/^\d+$/.test(debugPort) || Number(debugPort) < 1024 || Number(debugPort) > 65535)) {
  throw new Error('SIM_BROWSER_DEBUG_PORT must be a port between 1024 and 65535');
}
const browser = await chromium.launch({
  headless: !visible,
  args: debugPort ? [`--remote-debugging-port=${debugPort}`, '--remote-debugging-address=127.0.0.1'] : []
});
const context = await browser.newContext({ viewport: { width: 1440, height: 900 } });
const page = await context.newPage();
page.on('console', (message) => console.log(`[browser:${message.type()}] ${message.text()}`));
page.on('pageerror', (error) => console.error(`[browser:error] ${error.stack || error}`));
await page.goto(url, { waitUntil: 'domcontentloaded', timeout: 120_000 });
await page.waitForFunction(
  () => window.mujocoApp?.getState?.().model && window.mujocoBridge?.status?.().connected,
  null,
  { timeout: 180_000 },
);
console.log(`MuJoCo browser is ready at ${url}`);
process.send?.({ type: 'ready', url });

const stop = async () => {
  await browser.close();
  process.exit(0);
};
process.on('SIGINT', stop);
process.on('SIGTERM', stop);
await new Promise(() => {});

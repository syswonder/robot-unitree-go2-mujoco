import assert from 'node:assert/strict';
import { chromium } from 'playwright';

const browser = await chromium.launch({ headless: true, args: ['--no-sandbox'] });
try {
  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  await page.goto(`${process.env.WEB_TEST_URL ?? 'http://127.0.0.1:5182'}/?bridge=off&environment=scenesmith_house_185`);
  await page.waitForFunction(() => window.mujocoApp?.getState().simulationTime > 0.7, null, { timeout: 90000 });
  assert.equal(await page.locator('#robot-sensor-preview').count(), 0);
  const screenshot = await page.screenshot({ path: '/tmp/go2-house185-no-blank-panel.png' });
  const pixel = await page.evaluate(async (source) => {
    const image = new Image();
    image.src = source;
    await image.decode();
    const canvas = document.createElement('canvas');
    canvas.width = image.width;
    canvas.height = image.height;
    const context = canvas.getContext('2d');
    context.drawImage(image, 0, 0);
    return Array.from(context.getImageData(1120, 650, 1, 1).data);
  }, `data:image/png;base64,${screenshot.toString('base64')}`);
  assert.ok(pixel.slice(0, 3).some((channel) => channel < 240), 'Blank mirrored GUI rectangle remains');
  await page.locator('.lil-gui .title').filter({ hasText: /^Sensors$/ }).click();
  await page.getByRole('button', { name: 'Capture Front RGB-D', exact: true }).click();
  const images = await page.locator('#robot-sensor-preview canvas').evaluateAll((canvases) => canvases.map((canvas) => {
    const pixels = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
    const colors = new Set();
    for (let index = 0; index < pixels.length; index += 4) colors.add(`${pixels[index]},${pixels[index + 1]},${pixels[index + 2]}`);
    return { width: canvas.width, height: canvas.height, colors: colors.size };
  }));
  assert.equal(images.length, 2);
  assert.ok(images[0].colors > 100 && images[1].colors > 3, JSON.stringify(images));
  await page.screenshot({ path: '/tmp/go2-sensor-preview.png' });
  await page.getByRole('button', { name: 'Close sensor preview', exact: true }).click();
  assert.equal(await page.locator('#robot-sensor-preview').count(), 0);
  console.log(JSON.stringify({ defaultPixel: pixel, previewImages: images, closed: true }));
} finally {
  await browser.close();
}

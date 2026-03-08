const { chromium } = require('playwright');

async function main() {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
  const consoleErrors = [];

  page.on('console', (msg) => {
    if (msg.type() === 'error') {
      consoleErrors.push(msg.text());
    }
  });

  await page.goto('http://localhost:3456/index.html', { waitUntil: 'networkidle' });
  await page.waitForFunction(
    () => typeof window.getTrafficSnapshot === 'function' && window.getTrafficSnapshot().totals.intersections > 0
  );

  const canvas = page.locator('#sim-canvas');
  const box = await canvas.boundingBox();
  if (!box) {
    throw new Error('Canvas bounding box unavailable');
  }

  const readView = () => page.evaluate(() => window.getTrafficSnapshot().view);

  const before = await readView();
  await page.screenshot({ path: 'output/panzoom-manual-before.png' });

  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.5);
  await page.mouse.wheel(0, 520);
  await page.waitForTimeout(150);
  const afterZoom = await readView();

  await page.mouse.move(box.x + box.width * 0.72, box.y + box.height * 0.78);
  await page.mouse.down({ button: 'right' });
  await page.mouse.move(box.x + box.width * 0.9, box.y + box.height * 0.9, { steps: 12 });
  await page.mouse.up({ button: 'right' });
  await page.waitForTimeout(100);
  const afterRightPan = await readView();

  await page.keyboard.press('Enter');
  await page.mouse.move(box.x + box.width * 0.88, box.y + box.height * 0.86);
  await page.mouse.down({ button: 'left' });
  await page.mouse.move(box.x + box.width * 0.76, box.y + box.height * 0.76, { steps: 12 });
  await page.mouse.up({ button: 'left' });
  await page.waitForTimeout(100);
  const afterLeftPan = await readView();

  await page.screenshot({ path: 'output/panzoom-manual-after.png' });

  console.log(
    JSON.stringify(
      {
        before,
        afterZoom,
        afterRightPan,
        afterLeftPan,
        consoleErrors,
      },
      null,
      2
    )
  );

  await browser.close();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});

import {createHash} from 'node:crypto';
import {existsSync, readFileSync} from 'node:fs';
import {dirname, resolve} from 'node:path';
import {fileURLToPath} from 'node:url';

const repositoryRoot = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const sourceRevision = readFileSync(resolve(repositoryRoot, 'ROBONIX_SOURCE_REVISION'), 'utf8').trim();
const failures = [];

function fail(message) {
  failures.push(message);
}

function readJson(relativePath) {
  const absolutePath = resolve(repositoryRoot, relativePath);
  try {
    return JSON.parse(readFileSync(absolutePath, 'utf8'));
  } catch (error) {
    fail(`${relativePath}: ${error.message}`);
    return {};
  }
}

function requireString(value, location) {
  if (typeof value !== 'string' || value.trim() === '') {
    fail(`${location}: expected a non-empty string`);
    return false;
  }
  return true;
}

function requireFile(relativePath, location, root = repositoryRoot) {
  if (!requireString(relativePath, location)) {
    return;
  }
  if (!existsSync(resolve(root, relativePath))) {
    fail(`${location}: missing ${relativePath}`);
  }
}

const browserPath = 'validation/browser-qa-20260717.json';
const browser = readJson(browserPath);
requireString(browser.date, `${browserPath}.date`);
requireString(browser.site, `${browserPath}.site`);
for (const field of ['os', 'architecture', 'browser']) {
  requireString(browser.environment?.[field], `${browserPath}.environment.${field}`);
}
const viewportKeys = new Set(
  Array.isArray(browser.viewports)
    ? browser.viewports.map(({width, height}) => `${width}x${height}`)
    : [],
);
for (const viewport of ['1440x900', '390x844']) {
  if (!viewportKeys.has(viewport)) {
    fail(`${browserPath}.viewports: missing ${viewport}`);
  }
}
const routeFiles = new Map([
  ['/', 'docs/README.md'],
  ['/integration-guide/vendor-onboarding', 'docs/integration-guide/vendor-onboarding.md'],
  ['/developer-guide', 'docs/developer-guide.md'],
  ['/architecture/runtime-communication', 'docs/architecture/runtime-communication.md'],
  ['/reference/idl', 'docs/reference/idl.md'],
  ['/interface-catalog', 'docs/interface-catalog/index.md'],
]);
for (const [route, relativePath] of routeFiles) {
  if (!browser.pages?.includes(route)) {
    fail(`${browserPath}.pages: missing ${route}`);
  }
  requireFile(relativePath, `${browserPath}.pages[${route}]`);
}
for (const [index, screenshot] of (browser.local_screenshots ?? []).entries()) {
  if (!/^\/tmp\/.+\.png$/.test(screenshot.path ?? '')) {
    fail(`${browserPath}.local_screenshots[${index}].path: expected a local /tmp PNG path`);
  }
  if (!/^[0-9a-f]{64}$/.test(screenshot.sha256 ?? '')) {
    fail(`${browserPath}.local_screenshots[${index}].sha256: expected a SHA-256 digest`);
  }
  if (existsSync(screenshot.path)) {
    const actual = createHash('sha256').update(readFileSync(screenshot.path)).digest('hex');
    if (actual !== screenshot.sha256) {
      fail(`${browserPath}.local_screenshots[${index}]: local screenshot digest changed`);
    }
  }
}
requireString(browser.limitations, `${browserPath}.limitations`);

const deckPath = 'validation/runtime-communication-pptx-audit.json';
const deck = readJson(deckPath);
if (!/^[0-9a-f]{64}$/.test(deck.source_deck?.sha256 ?? '')) {
  fail(`${deckPath}.source_deck.sha256: expected a SHA-256 digest`);
}
requireString(deck.source_deck?.filename, `${deckPath}.source_deck.filename`);
requireString(deck.source_deck?.provenance, `${deckPath}.source_deck.provenance`);
if (deck.authoritative_source?.revision !== sourceRevision) {
  fail(`${deckPath}.authoritative_source.revision: expected ${sourceRevision}`);
}
if (!Array.isArray(deck.slides) || deck.slides.length !== deck.source_deck?.slides) {
  fail(`${deckPath}.slides: expected ${deck.source_deck?.slides ?? '<missing>'} slide records`);
}
for (const [index, slide] of (deck.slides ?? []).entries()) {
  if (slide.slide !== index + 1) {
    fail(`${deckPath}.slides[${index}].slide: expected ${index + 1}`);
  }
  requireString(slide.title, `${deckPath}.slides[${index}].title`);
  requireString(slide.verdict, `${deckPath}.slides[${index}].verdict`);
  if (!Array.isArray(slide.reconciled_claims) || slide.reconciled_claims.length === 0) {
    fail(`${deckPath}.slides[${index}].reconciled_claims: expected at least one claim`);
  }
  for (const [pageIndex, relativePath] of (slide.book_pages ?? []).entries()) {
    requireFile(relativePath, `${deckPath}.slides[${index}].book_pages[${pageIndex}]`);
  }
}

const sourceRoot = process.env.ROBONIX_SOURCE;
if (sourceRoot) {
  for (const [index, relativePath] of (deck.source_evidence ?? []).entries()) {
    requireFile(relativePath, `${deckPath}.source_evidence[${index}]`, sourceRoot);
  }
}

if (failures.length > 0) {
  console.error(`Validation-artifact check failed (${failures.length}):`);
  failures.forEach((failure) => console.error(`- ${failure}`));
  process.exit(1);
}

console.log(
  `Validation artifacts passed: browser routes=${routeFiles.size}, viewports=${viewportKeys.size}, deck slides=${(deck.slides ?? []).length}, source paths=${sourceRoot ? (deck.source_evidence ?? []).length : 'skipped'}`,
);

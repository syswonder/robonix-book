import {existsSync, readFileSync, readdirSync} from 'node:fs';
import {dirname, extname, join, relative, resolve} from 'node:path';

const root = process.cwd();
const docsRoot = join(root, 'docs');
const staticRoot = join(root, 'static');
const image = /!\[([^\]]*)\]\((<[^>]+>|[^\s)]+)(?:\s+["'][^)]*["'])?\)/g;
const failures = [];
let imageCount = 0;

function walk(directory) {
  return readdirSync(directory, {withFileTypes: true}).flatMap((entry) => {
    const path = join(directory, entry.name);
    return entry.isDirectory() ? walk(path) : [path];
  });
}

for (const source of walk(docsRoot).filter((path) => ['.md', '.mdx'].includes(extname(path)))) {
  const filename = relative(docsRoot, source);
  const text = readFileSync(source, 'utf8');
  for (const match of text.matchAll(image)) {
    imageCount += 1;
    const line = text.slice(0, match.index).split(/\r?\n/).length;
    const alt = match[1].trim();
    const target = match[2].replace(/^<|>$/g, '').split(/[?#]/, 1)[0];
    if (!alt) {
      failures.push(`${filename}:${line}: image has empty alternative text`);
    }
    if (/^(?:https?:|data:)/.test(target)) {
      continue;
    }
    const asset = target.startsWith('/')
      ? resolve(staticRoot, `.${target}`)
      : resolve(dirname(source), target);
    if (!existsSync(asset)) {
      failures.push(`${filename}:${line}: missing image asset ${target}`);
    }
  }
}

if (failures.length > 0) {
  console.error(`Documentation asset validation failed (${failures.length}):`);
  failures.forEach((failure) => console.error(`- ${failure}`));
  process.exit(1);
}

console.log(`Documentation asset validation passed: images=${imageCount}`);

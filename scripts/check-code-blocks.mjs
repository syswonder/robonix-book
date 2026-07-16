import {spawnSync} from 'node:child_process';
import {readFileSync, readdirSync} from 'node:fs';
import {extname, join, relative} from 'node:path';

import {loadAll as parseYaml} from 'js-yaml';
import {parse as parseToml} from 'smol-toml';

const docsRoot = join(process.cwd(), 'docs');
const checkedLanguages = new Set(['bash', 'js', 'json', 'python', 'toml', 'yaml']);
const counts = new Map();
const failures = [];

function walk(directory) {
  return readdirSync(directory, {withFileTypes: true}).flatMap((entry) => {
    const path = join(directory, entry.name);
    return entry.isDirectory() ? walk(path) : [path];
  });
}

function runSyntaxCommand(command, args, source) {
  const result = spawnSync(command, args, {
    input: source,
    encoding: 'utf8',
  });
  if (result.error) {
    throw result.error;
  }
  if (result.status !== 0) {
    throw new Error((result.stderr || result.stdout || `${command} exited ${result.status}`).trim());
  }
}

function checkBlock(language, source, filename) {
  switch (language) {
    case 'bash':
      runSyntaxCommand('bash', ['-n'], source);
      break;
    case 'js':
      runSyntaxCommand('node', ['--input-type=module', '--check'], source);
      break;
    case 'json':
      JSON.parse(source);
      break;
    case 'python':
      runSyntaxCommand(
        process.env.PYTHON || 'python3',
        ['-c', 'import ast, sys; ast.parse(sys.stdin.read())'],
        source,
      );
      break;
    case 'toml':
      parseToml(source);
      break;
    case 'yaml':
      parseYaml(source);
      break;
  }
}

for (const path of walk(docsRoot).filter((path) => ['.md', '.mdx'].includes(extname(path)))) {
  const filename = relative(docsRoot, path);
  const lines = readFileSync(path, 'utf8').split(/\r?\n/);

  for (let index = 0; index < lines.length; index += 1) {
    const opening = lines[index].match(/^ {0,3}(`{3,}|~{3,})([^\s`~]*)/);
    if (!opening) {
      continue;
    }

    const fence = opening[1];
    const language = opening[2] || '';
    const closing = new RegExp(`^ {0,3}${fence[0]}{${fence.length},}\\s*$`);
    const firstLine = index + 1;
    const body = [];
    index += 1;
    while (index < lines.length && !closing.test(lines[index])) {
      body.push(lines[index]);
      index += 1;
    }
    if (index === lines.length) {
      failures.push(`${filename}:${firstLine}: unclosed code fence`);
      break;
    }
    if (!language) {
      failures.push(`${filename}:${firstLine}: missing code-fence language`);
      continue;
    }
    if (!checkedLanguages.has(language)) {
      continue;
    }

    counts.set(language, (counts.get(language) || 0) + 1);
    try {
      checkBlock(language, body.join('\n'), `${filename}:${firstLine}`);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      failures.push(`${filename}:${firstLine}: ${language}: ${message}`);
    }
  }
}

if (failures.length > 0) {
  console.error(`Code-block validation failed (${failures.length}):`);
  failures.forEach((failure) => console.error(`- ${failure}`));
  process.exit(1);
}

const summary = [...counts.entries()]
  .sort(([left], [right]) => left.localeCompare(right))
  .map(([language, count]) => `${language}=${count}`)
  .join(', ');
console.log(`Code-block syntax validation passed: ${summary}`);

#!/usr/bin/env node

import {createHash} from 'node:crypto';
import {existsSync, readFileSync, readdirSync, statSync} from 'node:fs';
import {dirname, extname, join, relative, resolve, sep} from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';

import {dump as stringifyYaml, load as parseYaml} from 'js-yaml';

const scriptDirectory = dirname(fileURLToPath(import.meta.url));
const repositoryRoot = resolve(scriptDirectory, '..');
const docsRoot = join(repositoryRoot, 'docs');
const evidencePath = join(repositoryRoot, 'validation', 'page-evidence.yaml');
const revisionPath = join(repositoryRoot, 'ROBONIX_SOURCE_REVISION');
const allowedExcludedPaths = new Set([
  'docs/reference/contracts.md',
  'docs/reference/idl.md',
]);

const argumentsList = process.argv.slice(2);
const flags = new Set();
let explicitSource;
for (let index = 0; index < argumentsList.length; index += 1) {
  const argument = argumentsList[index];
  if (argument === '--source') {
    explicitSource = argumentsList[index + 1];
    index += 1;
    if (!explicitSource) {
      console.error('--source requires a path');
      process.exit(2);
    }
    continue;
  }
  if (['--inventory', '--refresh', '--strict-evidence', '--require-source'].includes(argument)) {
    flags.add(argument);
    continue;
  }
  console.error(`Unknown argument: ${argument}`);
  process.exit(2);
}

function walk(directory) {
  return readdirSync(directory, {withFileTypes: true}).flatMap((entry) => {
    const path = join(directory, entry.name);
    return entry.isDirectory() ? walk(path) : [path];
  });
}

function toRepositoryPath(path) {
  return relative(repositoryRoot, path).split(sep).join('/');
}

function parseFencesText(contents, displayPath) {
  const lines = contents.split(/\r?\n/);
  const blocks = [];
  for (let index = 0; index < lines.length; index += 1) {
    const opening = lines[index].match(/^ {0,3}(`{3,}|~{3,})([^\s`~]*)\s*(.*)$/);
    if (!opening) {
      continue;
    }

    const fence = opening[1];
    const closing = new RegExp(`^ {0,3}${fence[0]}{${fence.length},}\\s*$`);
    const firstLine = index + 1;
    const body = [];
    index += 1;
    while (index < lines.length && !closing.test(lines[index])) {
      body.push(lines[index]);
      index += 1;
    }
    if (index === lines.length) {
      throw new Error(`${displayPath}:${firstLine}: unclosed code fence`);
    }
    blocks.push({
      line: firstLine,
      endLine: index + 1,
      language: opening[2] || '',
      meta: opening[3].trim(),
      body: body.join('\n'),
    });
  }
  return blocks;
}

function parseFences(path) {
  return parseFencesText(readFileSync(path, 'utf8'), toRepositoryPath(path));
}

function fenceDigest(blocks) {
  const stableBlocks = blocks.map(({line, endLine, language, meta, body}) => ({
    line,
    endLine,
    language,
    meta,
    body,
  }));
  return createHash('sha256').update(JSON.stringify(stableBlocks)).digest('hex');
}

function blockFingerprint(block) {
  return createHash('sha256')
    .update(JSON.stringify({language: block.language, meta: block.meta, body: block.body}))
    .digest('hex');
}

const markdownPaths = walk(docsRoot)
  .filter((path) => ['.md', '.mdx'].includes(extname(path)))
  .sort();
const inventory = new Map(
  markdownPaths.map((path) => {
    const blocks = parseFences(path);
    return [toRepositoryPath(path), {blocks, digest: fenceDigest(blocks)}];
  }),
);
const docsInventoryDigest = createHash('sha256')
  .update(JSON.stringify([...inventory.entries()].map(([path, page]) => [path, page.digest])))
  .digest('hex');

if (flags.has('--inventory')) {
  const printable = Object.fromEntries(
    [...inventory.entries()].map(([path, page]) => [
      path,
      {
        fence_digest: page.digest,
        blocks: page.blocks.map(({line, language, meta, body}) => ({
          line,
          language,
          ...(meta ? {meta} : {}),
          preview: body.split('\n').find((entry) => entry.trim())?.trim().slice(0, 120) || '',
        })),
      },
    ]),
  );
  console.log(JSON.stringify(printable, null, 2));
  process.exit(0);
}

const failures = [];
const warnings = [];
const evidenceGaps = [];

function fail(message) {
  failures.push(message);
}

function requireNonEmptyString(value, location) {
  if (typeof value !== 'string' || value.trim() === '') {
    fail(`${location}: expected a non-empty string`);
    return false;
  }
  return true;
}

if (!existsSync(evidencePath)) {
  console.error(`Page-evidence validation failed: missing ${toRepositoryPath(evidencePath)}`);
  process.exit(1);
}

let matrix;
try {
  matrix = parseYaml(readFileSync(evidencePath, 'utf8'));
} catch (error) {
  console.error(`Page-evidence validation failed: cannot parse ${toRepositoryPath(evidencePath)}`);
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
}

if (!matrix || typeof matrix !== 'object') {
  console.error(`Page-evidence validation failed: ${toRepositoryPath(evidencePath)} is not a mapping`);
  process.exit(1);
}

if (flags.has('--refresh')) {
  const excluded = new Set((matrix.exclusions ?? []).map((entry) => entry.path));
  const refreshNotes = [];
  matrix.pages ??= {};

  // Once the ledger is tracked, use its committed page records as the trusted
  // refresh baseline. This prevents a changed fence from retaining old evidence
  // merely because its working-tree fingerprint was pre-edited to match.
  const evidenceRepositoryPath = toRepositoryPath(evidencePath);
  const committedLedgerResult = spawnSync(
    'git',
    ['-C', repositoryRoot, 'show', `HEAD:${evidenceRepositoryPath}`],
    {encoding: 'utf8'},
  );
  let committedPages;
  if (committedLedgerResult.status === 0) {
    let committedMatrix;
    try {
      committedMatrix = parseYaml(committedLedgerResult.stdout);
    } catch (error) {
      console.error('Refresh refused: the committed page-evidence ledger is not valid YAML');
      console.error(error instanceof Error ? error.message : String(error));
      process.exit(1);
    }
    committedPages = committedMatrix?.pages ?? {};
    for (const [path, page] of Object.entries(committedPages)) {
      const committedPageResult = spawnSync(
        'git',
        ['-C', repositoryRoot, 'show', `HEAD:${path}`],
        {encoding: 'utf8'},
      );
      if (committedPageResult.status !== 0) {
        console.error(`Refresh refused: committed ledger page ${path} is absent from HEAD`);
        process.exit(1);
      }
      const committedBlocks = parseFencesText(committedPageResult.stdout, `${path}@HEAD`);
      const expectedDigest = fenceDigest(committedBlocks);
      const expectedFingerprints = Object.fromEntries(
        committedBlocks.map((block) => [String(block.line), blockFingerprint(block)]),
      );
      if (
        page.fence_digest !== expectedDigest ||
        JSON.stringify(page.block_fingerprints ?? {}) !== JSON.stringify(expectedFingerprints)
      ) {
        console.error(
          `Refresh refused: committed ledger fingerprints for ${path} do not match the committed page`,
        );
        process.exit(1);
      }
    }
  } else {
    refreshNotes.push(
      `${evidenceRepositoryPath}: no committed ledger exists; initial bootstrap trusts the reviewed working ledger`,
    );
  }

  for (const [path, current] of inventory.entries()) {
    if (excluded.has(path)) {
      continue;
    }
    let page = matrix.pages[path];
    if (!page) {
      page = {
        source_revision: matrix.source?.revision ?? '',
        source_evidence: [],
        fence_digest: current.digest,
        block_fingerprints: {},
        blocks: [],
      };
      matrix.pages[path] = page;
      refreshNotes.push(`${path}: new page needs source provenance review`);
    }

    const committedPage = committedPages?.[path];
    if (committedPages && !committedPage) {
      page.blocks = [];
      page.block_fingerprints = {};
    }
    const refreshBase = committedPage ?? page;

    if (refreshBase.fence_digest === current.digest) {
      page.fence_digest = current.digest;
      page.block_fingerprints = Object.fromEntries(
        current.blocks.map((block) => [String(block.line), blockFingerprint(block)]),
      );
      continue;
    }

    const currentByFingerprint = new Map();
    for (const block of current.blocks) {
      const fingerprint = blockFingerprint(block);
      const matches = currentByFingerprint.get(fingerprint) ?? [];
      matches.push(block.line);
      currentByFingerprint.set(fingerprint, matches);
    }

    const oldFingerprints = refreshBase.block_fingerprints ?? {};
    const oldFingerprintCounts = new Map();
    for (const fingerprint of Object.values(oldFingerprints)) {
      oldFingerprintCounts.set(fingerprint, (oldFingerprintCounts.get(fingerprint) ?? 0) + 1);
    }
    const occurrenceByFingerprint = new Map();
    const refreshedGroups = [];
    const usedCurrentLines = new Set();
    for (const group of refreshBase.blocks ?? []) {
      if (group.profile === 'needs-review') {
        continue;
      }
      const refreshedLines = [];
      for (const oldLine of group.lines ?? []) {
        const fingerprint = oldFingerprints[String(oldLine)];
        let newLine;
        if (fingerprint) {
          const currentMatches = currentByFingerprint.get(fingerprint) ?? [];
          if (oldFingerprintCounts.get(fingerprint) === 1 && currentMatches.length === 1) {
            const occurrence = occurrenceByFingerprint.get(fingerprint) ?? 0;
            newLine = currentMatches[occurrence];
            occurrenceByFingerprint.set(fingerprint, occurrence + 1);
          } else {
            refreshNotes.push(
              `${path}:${oldLine}: duplicate-content fence is ambiguous and needs classification review`,
            );
          }
        }
        if (newLine === undefined || usedCurrentLines.has(newLine)) {
          refreshNotes.push(`${path}:${oldLine}: changed or removed fence needs classification review`);
          continue;
        }
        usedCurrentLines.add(newLine);
        refreshedLines.push(newLine);
      }
      if (refreshedLines.length > 0) {
        refreshedGroups.push({
          profile: group.profile,
          lines: refreshedLines.sort((left, right) => left - right),
          ...(group.validation ? {validation: group.validation} : {}),
        });
      }
    }

    const needsReview = current.blocks
      .map((block) => block.line)
      .filter((line) => !usedCurrentLines.has(line));
    if (needsReview.length > 0) {
      refreshedGroups.push({profile: 'needs-review', lines: needsReview});
      refreshNotes.push(`${path}:${needsReview.join(',')}: new or changed fences need classification review`);
    }

    page.fence_digest = current.digest;
    page.block_fingerprints = Object.fromEntries(
      current.blocks.map((block) => [String(block.line), blockFingerprint(block)]),
    );
    page.blocks = refreshedGroups;
  }

  for (const path of Object.keys(matrix.pages)) {
    if (!inventory.has(path) || excluded.has(path)) {
      refreshNotes.push(`${path}: stale matrix page needs manual removal or exclusion review`);
    }
  }

  if (refreshNotes.length > 0) {
    console.error(`Refresh review notes (${refreshNotes.length}):`);
    refreshNotes.forEach((note) => console.error(`- ${note}`));
  }
  process.stdout.write(
    stringifyYaml(matrix, {
      lineWidth: -1,
      noRefs: true,
      noCompatMode: true,
      sortKeys: false,
    }),
  );
  process.exit(0);
}

const pinnedRevision = readFileSync(revisionPath, 'utf8').trim();
if (!/^[0-9a-f]{40}$/.test(pinnedRevision)) {
  fail('ROBONIX_SOURCE_REVISION: expected a full 40-character Git revision');
}
if (matrix.schema_version !== 2) {
  fail('schema_version: expected 2');
}
if (matrix.source?.revision !== pinnedRevision) {
  fail(`source.revision: expected ${pinnedRevision}, found ${matrix.source?.revision ?? '<missing>'}`);
}
if (matrix.source?.branch !== 'dev-next') {
  fail(`source.branch: expected dev-next, found ${matrix.source?.branch ?? '<missing>'}`);
}

const evidenceRuns = matrix.evidence_runs ?? {};
const allowedRunScopes = new Set(['syntax', 'parser', 'execution', 'integration', 'hardware']);
for (const [runName, run] of Object.entries(evidenceRuns)) {
  const location = `evidence_runs.${runName}`;
  requireNonEmptyString(run.type, `${location}.type`);
  if (!allowedRunScopes.has(run.scope)) {
    fail(`${location}.scope: expected syntax, parser, execution, integration, or hardware`);
  }
  if (run.scope === 'syntax' && run.binds_to_docs !== true) {
    fail(`${location}.binds_to_docs: syntax evidence must bind to the exact handbook fence inventory`);
  }
  if (run.binds_to_docs === true && run.docs_inventory_digest !== docsInventoryDigest) {
    fail(
      `${location}.docs_inventory_digest: handbook fences changed; expected ${docsInventoryDigest}, found ${run.docs_inventory_digest ?? '<missing>'}`,
    );
  }
  if (run.historical !== undefined && typeof run.historical !== 'boolean') {
    fail(`${location}.historical: expected a boolean when present`);
  }
  const historical = run.historical === true;
  if (historical) {
    if (!/^[0-9a-f]{40}$/.test(run.source_revision ?? '')) {
      fail(`${location}.source_revision: historical evidence needs its exact 40-character Git revision`);
    } else if (run.source_revision === pinnedRevision) {
      fail(`${location}.historical: the run revision must predate the current source pin`);
    }
  } else if (run.source_revision !== pinnedRevision) {
    fail(`${location}.source_revision: expected ${pinnedRevision}`);
  }
  requireNonEmptyString(run.environment?.os, `${location}.environment.os`);
  requireNonEmptyString(run.environment?.architecture, `${location}.environment.architecture`);
  if (!Array.isArray(run.commands) || run.commands.length === 0) {
    fail(`${location}.commands: expected at least one exact command record`);
  } else {
    for (const [index, command] of run.commands.entries()) {
      const commandLocation = `${location}.commands[${index}]`;
      requireNonEmptyString(command.cwd, `${commandLocation}.cwd`);
      requireNonEmptyString(command.command, `${commandLocation}.command`);
      if (!Number.isInteger(command.exit_code)) {
        fail(`${commandLocation}.exit_code: expected an integer`);
      }
      requireNonEmptyString(command.result, `${commandLocation}.result`);
    }
  }
  requireNonEmptyString(run.result, `${location}.result`);
  if (!['passed', 'failed', 'blocked', 'pending'].includes(run.status)) {
    fail(`${location}.status: expected passed, failed, blocked, or pending`);
  }
  if (historical && !['failed', 'blocked'].includes(run.status)) {
    fail(`${location}.historical: only failed or blocked runs may be retained across source revisions`);
  }
  if (run.status === 'passed' && run.commands?.some((command) => command.exit_code !== 0)) {
    fail(`${location}: passed runs cannot contain a non-zero command result`);
  }
}

const profiles = matrix.profiles ?? {};
const allowedClassifications = new Set([
  'runnable',
  'safety-gated',
  'illustrative',
  'partial',
  'output',
  'reference',
]);
const allowedPresentations = new Set(['complete', 'illustrative', 'partial', 'output', 'reference']);
for (const [profileName, profile] of Object.entries(profiles)) {
  const location = `profiles.${profileName}`;
  if (!allowedClassifications.has(profile.classification)) {
    fail(`${location}.classification: unknown classification ${profile.classification ?? '<missing>'}`);
  }
  if (!allowedPresentations.has(profile.presentation)) {
    fail(`${location}.presentation: unknown presentation ${profile.presentation ?? '<missing>'}`);
  }
  const requirement = profile.evidence_requirement;
  if (['runnable', 'safety-gated'].includes(profile.classification)) {
    if (!['command-contract', 'runtime-smoke', 'end-to-end', 'staged-non-motion'].includes(requirement)) {
      fail(`${location}.evidence_requirement: unknown runnable evidence requirement ${requirement ?? '<missing>'}`);
    }
  } else if (requirement !== 'not-applicable') {
    fail(`${location}.evidence_requirement: non-runnable profiles must use not-applicable`);
  }
  if (profile.classification === 'safety-gated') {
    const hardware = profile.real_hardware_acceptance;
    if (!hardware || typeof hardware !== 'object') {
      fail(`${location}.real_hardware_acceptance: required for safety-gated blocks`);
    } else {
      if (!Array.isArray(hardware.required_conditions) || hardware.required_conditions.length === 0) {
        fail(`${location}.real_hardware_acceptance.required_conditions: expected a non-empty list`);
      }
      if (!Array.isArray(hardware.required_evidence) || hardware.required_evidence.length === 0) {
        fail(`${location}.real_hardware_acceptance.required_evidence: expected a non-empty list`);
      }
    }
  }
}

const exclusions = matrix.exclusions ?? [];
if (!Array.isArray(exclusions)) {
  fail('exclusions: expected a list');
}
const excludedPaths = new Set();
for (const [index, exclusion] of (Array.isArray(exclusions) ? exclusions : []).entries()) {
  const location = `exclusions[${index}]`;
  if (requireNonEmptyString(exclusion.path, `${location}.path`)) {
    if (excludedPaths.has(exclusion.path)) {
      fail(`${location}.path: duplicate exclusion ${exclusion.path}`);
    }
    excludedPaths.add(exclusion.path);
    if (!inventory.has(exclusion.path)) {
      fail(`${location}.path: excluded page does not exist under docs/`);
    }
  }
  requireNonEmptyString(exclusion.reason, `${location}.reason`);
  if (exclusion.source_revision !== pinnedRevision) {
    fail(`${location}.source_revision: expected ${pinnedRevision}`);
  }
  if (!Array.isArray(exclusion.generated_by) || exclusion.generated_by.length === 0) {
    fail(`${location}.generated_by: expected at least one pinned source path`);
  }
}
for (const path of excludedPaths) {
  if (!allowedExcludedPaths.has(path)) {
    fail(`exclusions: ${path} is not one of the two generated reference pages`);
  }
}
for (const path of allowedExcludedPaths) {
  if (!excludedPaths.has(path)) {
    fail(`exclusions: required generated-page exclusion ${path} is missing`);
  }
}
if (excludedPaths.size !== allowedExcludedPaths.size) {
  fail(`exclusions: expected exactly ${allowedExcludedPaths.size} generated reference pages`);
}

const pages = matrix.pages ?? {};
const activePaths = [...inventory.keys()].filter((path) => !excludedPaths.has(path)).sort();
const matrixPaths = Object.keys(pages).sort();
for (const path of activePaths.filter((path) => !Object.hasOwn(pages, path))) {
  fail(`${path}: active handbook page is missing from the evidence matrix`);
}
for (const path of matrixPaths.filter((path) => !activePaths.includes(path))) {
  fail(`${path}: matrix entry is not an active handbook page`);
}

const allowedSourceKinds = new Set([
  'implementation',
  'api',
  'config',
  'protocol',
  'test',
  'source-documentation',
]);
const sourcePaths = new Set();
let blockCount = 0;
const classificationCounts = new Map();
const evidenceStatusCounts = new Map();
const acceptedScopesByRequirement = {
  'command-contract': new Set(['parser', 'execution', 'integration', 'hardware']),
  'runtime-smoke': new Set(['execution', 'integration', 'hardware']),
  'end-to-end': new Set(['integration', 'hardware']),
  'staged-non-motion': new Set(['integration', 'hardware']),
};

function validateRunReferences(runNames, location, requirement, acceptedScopesOverride) {
  if (!Array.isArray(runNames) || runNames.length === 0) {
    fail(`${location}: expected at least one evidence run`);
    return {known: [], qualifyingPassed: []};
  }
  const known = [];
  for (const [index, runName] of runNames.entries()) {
    if (typeof runName !== 'string' || !evidenceRuns[runName]) {
      fail(`${location}[${index}]: unknown evidence run ${runName ?? '<missing>'}`);
    } else {
      known.push([runName, evidenceRuns[runName]]);
    }
  }
  const acceptedScopes = acceptedScopesOverride ?? acceptedScopesByRequirement[requirement] ?? new Set();
  return {
    known,
    qualifyingPassed: known.filter(
      ([, run]) => run.historical !== true && run.status === 'passed' && acceptedScopes.has(run.scope),
    ),
  };
}

for (const path of matrixPaths) {
  if (!inventory.has(path) || excludedPaths.has(path)) {
    continue;
  }
  const page = pages[path];
  const location = `pages.${path}`;
  if (!page || typeof page !== 'object') {
    fail(`${location}: expected a mapping`);
    continue;
  }
  if (page.source_revision !== pinnedRevision) {
    fail(`${location}.source_revision: expected ${pinnedRevision}`);
  }
  if (!Array.isArray(page.source_evidence) || page.source_evidence.length === 0) {
    fail(`${location}.source_evidence: expected at least one authoritative dev-next path`);
  } else {
    for (const [index, evidence] of page.source_evidence.entries()) {
      const evidenceLocation = `${location}.source_evidence[${index}]`;
      if (!allowedSourceKinds.has(evidence.kind)) {
        fail(`${evidenceLocation}.kind: unknown source kind ${evidence.kind ?? '<missing>'}`);
      }
      if (requireNonEmptyString(evidence.path, `${evidenceLocation}.path`)) {
        if (evidence.path.startsWith('/') || evidence.path.split('/').includes('..')) {
          fail(`${evidenceLocation}.path: source paths must be repository-relative`);
        } else {
          sourcePaths.add(evidence.path);
        }
      }
      requireNonEmptyString(evidence.claim, `${evidenceLocation}.claim`);
    }
  }

  const actual = inventory.get(path);
  if (page.fence_digest !== actual.digest) {
    fail(
      `${location}.fence_digest: page fences changed; expected ${actual.digest}, found ${page.fence_digest ?? '<missing>'}`,
    );
  }

  if (!Array.isArray(page.blocks)) {
    fail(`${location}.blocks: expected a list (use [] when the page has no fences)`);
    continue;
  }
  if (!page.block_fingerprints || typeof page.block_fingerprints !== 'object') {
    fail(`${location}.block_fingerprints: expected a line-to-content-fingerprint mapping`);
  } else {
    for (const block of actual.blocks) {
      const recorded = page.block_fingerprints[String(block.line)];
      const expected = blockFingerprint(block);
      if (recorded !== expected) {
        fail(
          `${path}:${block.line}: fence fingerprint changed; expected ${expected}, found ${recorded ?? '<missing>'}`,
        );
      }
    }
    for (const line of Object.keys(page.block_fingerprints)) {
      if (!actual.blocks.some((block) => String(block.line) === String(line))) {
        fail(`${path}:${line}: stale fence fingerprint`);
      }
    }
  }
  const actualLines = new Set(actual.blocks.map((block) => block.line));
  const classifiedLines = new Map();
  for (const [groupIndex, group] of page.blocks.entries()) {
    const groupLocation = `${location}.blocks[${groupIndex}]`;
    const profile = profiles[group.profile];
    if (!profile) {
      fail(`${groupLocation}.profile: unknown profile ${group.profile ?? '<missing>'}`);
      continue;
    }
    if (!Array.isArray(group.lines) || group.lines.length === 0) {
      fail(`${groupLocation}.lines: expected a non-empty line-number list`);
      continue;
    }
    const normalizedLines = [...group.lines].sort((left, right) => left - right);
    if (JSON.stringify(normalizedLines) !== JSON.stringify(group.lines)) {
      fail(`${groupLocation}.lines: line numbers must be sorted`);
    }
    for (const line of group.lines) {
      if (!Number.isInteger(line) || line < 1) {
        fail(`${groupLocation}.lines: ${line} is not a positive integer`);
        continue;
      }
      if (classifiedLines.has(line)) {
        fail(`${path}:${line}: fence classified more than once`);
        continue;
      }
      classifiedLines.set(line, group.profile);
      if (!actualLines.has(line)) {
        fail(`${path}:${line}: matrix line is not a current fence opening`);
      }
      blockCount += 1;
      classificationCounts.set(
        profile.classification,
        (classificationCounts.get(profile.classification) ?? 0) + 1,
      );
    }
    if (['runnable', 'safety-gated'].includes(profile.classification)) {
      const validation = group.validation;
      if (!validation || typeof validation !== 'object') {
        fail(`${groupLocation}.validation: runnable blocks require a block-group evidence record`);
        evidenceGaps.push({
          path,
          lines: group.lines,
          profile: group.profile,
          status: 'missing',
          gap: 'block-group evidence record is missing',
        });
        continue;
      }
      const allowedStatuses = profile.classification === 'safety-gated'
        ? ['staged-passed', 'passed', 'failed', 'blocked', 'pending']
        : ['passed', 'failed', 'blocked', 'pending'];
      if (!allowedStatuses.includes(validation.status)) {
        fail(`${groupLocation}.validation.status: unknown status ${validation.status ?? '<missing>'}`);
      }
      evidenceStatusCounts.set(
        validation.status ?? 'missing',
        (evidenceStatusCounts.get(validation.status ?? 'missing') ?? 0) + group.lines.length,
      );
      const evidenceRequirement = validation.requirement ?? profile.evidence_requirement;
      if (!acceptedScopesByRequirement[evidenceRequirement]) {
        fail(`${groupLocation}.validation.requirement: unknown evidence requirement ${evidenceRequirement ?? '<missing>'}`);
      }
      const runValidation = validateRunReferences(
        validation.runs,
        `${groupLocation}.validation.runs`,
        evidenceRequirement,
      );
      let accepted = false;
      if (profile.classification === 'runnable') {
        accepted = validation.status === 'passed' && runValidation.qualifyingPassed.length > 0;
        if (validation.status === 'passed' && runValidation.qualifyingPassed.length === 0) {
          fail(
            `${groupLocation}.validation: passed runnable evidence needs a passed ${evidenceRequirement} run; syntax-only and failed runs do not qualify`,
          );
        }
      } else {
        const nonMotion = validation.last_non_motion_validation;
        if (!nonMotion || typeof nonMotion !== 'object') {
          fail(`${groupLocation}.validation.last_non_motion_validation: required for safety-gated blocks`);
        } else {
          if (!['passed', 'failed', 'blocked', 'pending'].includes(nonMotion.status)) {
            fail(`${groupLocation}.validation.last_non_motion_validation.status: unknown status`);
          }
          const nonMotionRuns = validateRunReferences(
            nonMotion.runs,
            `${groupLocation}.validation.last_non_motion_validation.runs`,
            'staged-non-motion',
          );
          if (nonMotion.status === 'passed' && nonMotionRuns.qualifyingPassed.length === 0) {
            fail(`${groupLocation}.validation.last_non_motion_validation: passed status needs a passed integration or hardware run`);
          }
          const hardware = validation.real_hardware_acceptance;
          let hardwareRuns = {known: [], qualifyingPassed: []};
          if (!hardware || typeof hardware !== 'object') {
            fail(`${groupLocation}.validation.real_hardware_acceptance: required for safety-gated blocks`);
          } else if (!['passed', 'failed', 'blocked', 'pending'].includes(hardware.status)) {
            fail(`${groupLocation}.validation.real_hardware_acceptance.status: unknown status`);
          } else if (hardware.status === 'passed') {
            hardwareRuns = validateRunReferences(
              hardware.runs,
              `${groupLocation}.validation.real_hardware_acceptance.runs`,
              'real-hardware',
              new Set(['hardware']),
            );
            if (hardwareRuns.qualifyingPassed.length === 0) {
              fail(`${groupLocation}.validation.real_hardware_acceptance: passed status needs a passed hardware-scope run`);
            }
          }
          accepted =
            ['staged-passed', 'passed'].includes(validation.status) &&
            nonMotion.status === 'passed' &&
            nonMotionRuns.qualifyingPassed.length > 0 &&
            (
              validation.status !== 'passed' ||
              (hardware?.status === 'passed' && hardwareRuns.qualifyingPassed.length > 0)
            );
        }
      }
      if (!accepted) {
        requireNonEmptyString(validation.gap, `${groupLocation}.validation.gap`);
        evidenceGaps.push({
          path,
          lines: group.lines,
          profile: group.profile,
          status: validation.status ?? 'missing',
          gap: validation.gap ?? 'required evidence is not passed',
        });
      }
    }
  }
  for (const line of [...actualLines].filter((line) => !classifiedLines.has(line)).sort((a, b) => a - b)) {
    fail(`${path}:${line}: fenced block is unclassified`);
  }
}

for (const exclusion of Array.isArray(exclusions) ? exclusions : []) {
  for (const path of exclusion.generated_by ?? []) {
    if (typeof path === 'string' && path !== '') {
      sourcePaths.add(path);
    }
  }
}

function git(sourceRoot, args) {
  return spawnSync('git', ['-C', sourceRoot, ...args], {encoding: 'utf8'});
}

let sourceRoot;
if (explicitSource) {
  sourceRoot = resolve(explicitSource);
} else if (process.env.ROBONIX_SOURCE) {
  sourceRoot = resolve(process.env.ROBONIX_SOURCE);
} else {
  const sibling = resolve(repositoryRoot, '..', 'robonix-dev-next');
  if (existsSync(sibling) && statSync(sibling).isDirectory()) {
    sourceRoot = sibling;
  }
}

let validatedSourcePaths = 0;
let sourceHead;
if (!sourceRoot) {
  const message =
    'source-path existence was not checked; set ROBONIX_SOURCE or pass --source (use --require-source to make this fatal)';
  if (flags.has('--require-source')) {
    fail(message);
  } else {
    warnings.push(message);
  }
} else {
  const revisionCheck = git(sourceRoot, ['cat-file', '-e', `${pinnedRevision}^{commit}`]);
  if (revisionCheck.status !== 0) {
    fail(`source checkout ${sourceRoot} does not contain pinned revision ${pinnedRevision}`);
  } else {
    const headResult = git(sourceRoot, ['rev-parse', 'HEAD']);
    if (headResult.status === 0) {
      sourceHead = headResult.stdout.trim();
      if (sourceHead !== pinnedRevision) {
        const message =
          `source HEAD ${sourceHead} differs from pin ${pinnedRevision}; paths were checked in the pinned Git tree`;
        if (flags.has('--require-source')) {
          fail(message);
        } else {
          warnings.push(message);
        }
      }
    } else if (flags.has('--require-source')) {
      fail(`source checkout ${sourceRoot} has no readable HEAD`);
    }
    if (flags.has('--require-source')) {
      const worktreeResult = git(sourceRoot, ['status', '--porcelain=v1', '--untracked-files=all']);
      if (worktreeResult.status !== 0) {
        fail(`source checkout ${sourceRoot} worktree status could not be read`);
      } else if (worktreeResult.stdout.trim() !== '') {
        fail(`source checkout ${sourceRoot} is not clean`);
      }
    }
    for (const path of [...sourcePaths].sort()) {
      // ls-tree validates files, trees, and gitlinks without requiring the
      // submodule commit object to be present in the parent repository.
      const result = git(sourceRoot, ['ls-tree', '--name-only', pinnedRevision, '--', path]);
      const exactMatch = result.stdout
        .split(/\r?\n/)
        .some((entry) => entry === path);
      if (result.status !== 0 || !exactMatch) {
        fail(`source:${pinnedRevision}:${path}: authoritative path does not exist in the pinned tree`);
      } else {
        validatedSourcePaths += 1;
      }
    }
  }
}

if (flags.has('--strict-evidence')) {
  for (const gap of evidenceGaps) {
    fail(
      `${gap.path}:${gap.lines.join(',')}: ${gap.profile} is ${gap.status}; ${gap.gap}`,
    );
  }
}

if (warnings.length > 0) {
  console.warn(`Page-evidence warnings (${warnings.length}):`);
  warnings.forEach((warning) => console.warn(`- ${warning}`));
}

if (failures.length > 0) {
  console.error(`Page-evidence validation failed (${failures.length}):`);
  failures.forEach((failure) => console.error(`- ${failure}`));
  process.exit(1);
}

const classSummary = [...classificationCounts.entries()]
  .sort(([left], [right]) => left.localeCompare(right))
  .map(([classification, count]) => `${classification}=${count}`)
  .join(', ');
const evidenceSummary = [...evidenceStatusCounts.entries()]
  .sort(([left], [right]) => left.localeCompare(right))
  .map(([status, count]) => `${status}=${count}`)
  .join(', ');
const sourceSummary = sourceRoot
  ? `source-paths=${validatedSourcePaths}, source-head=${sourceHead?.slice(0, 12) ?? 'unknown'}`
  : 'source-paths=skipped';
console.log(
  `Page-evidence validation passed: pages=${activePaths.length}, excluded=${excludedPaths.size}, blocks=${blockCount}, ${classSummary}; evidence=${evidenceSummary || 'not-applicable'}; ${sourceSummary}`,
);

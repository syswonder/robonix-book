#!/usr/bin/env node

import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';

const binary = process.env.RBNX_BIN;
if (!binary) {
  console.error('RBNX_BIN must point to the rbnx binary under test');
  process.exit(2);
}

const probes = [
  ['build', '-p', '.', '--clean', '--no-update-check'],
  ['build', '-f', 'robonix_manifest.yaml', '--no-update-check'],
  ['start', '-p', '.', '--endpoint', '127.0.0.1:50051'],
  [
    'start',
    '-p',
    '.',
    '--config',
    'local-config.yaml',
    '--set',
    'timeout_s=30',
    '--manifest',
    'package_manifest.jetson-native.yaml',
  ],
  ['boot', '-v', '-f', 'robonix_manifest.yaml', '--no-update-check'],
  ['boot', '-f', 'robonix_manifest.yaml', '--log-dir', 'rbnx-boot/logs', '--skip-system'],
  ['update', '-f', 'robonix_manifest.yaml'],
  ['update', '-p', 'rbnx-boot/cache/service-map-rbnx'],
  ['shutdown', '-f', 'robonix_manifest.yaml'],
  ['clean', '-f', 'robonix_manifest.yaml', '--cache'],
  ['validate', '.'],
  ['config', '--show'],
  ['codegen', '-p', '.', '--mcp', '--ros2', '--clean'],
  ['docs'],
  ['setup', '/absolute/path/to/robonix'],
  ['path', 'root'],
  ['path', 'rust'],
  ['path', 'capabilities'],
  ['path', 'interfaces-lib'],
  ['path', 'runtime-proto'],
  ['path', 'robonix-api'],
  ['caps', '-v'],
  ['caps', '--json'],
  ['contracts'],
  ['describe', '--provider', 'grasp_block'],
  ['describe', '--json'],
  ['tools'],
  ['channels'],
  ['inspect'],
  ['chat'],
  ['init', 'robot-acme-rover'],
  ['package-new', 'my_camera', '--type', 'primitive'],
  ['package-new', 'my_mapper', '--type', 'service', '--path', './packages/my_mapper'],
  ['ask', 'describe current runtime state'],
  ['logs', '--list-tags'],
  ['logs', '-d', 'rbnx-boot/logs', '-t', 'mapping', '--json'],
  ['logs', '-t', 'mapping', '-t', 'nav2', '-l', 'info', '-f'],
];

const failures = [];
for (const args of probes) {
  const commandArgs = [...args, '--help'];
  const result = spawnSync(resolve(binary), commandArgs, {
    cwd: process.cwd(),
    encoding: 'utf8',
  });
  if (result.status !== 0) {
    failures.push({
      command: ['rbnx', ...commandArgs].join(' '),
      status: result.status,
      stderr: result.stderr.trim(),
    });
  }
}

if (failures.length > 0) {
  for (const failure of failures) {
    console.error(`FAILED [${failure.status}]: ${failure.command}`);
    if (failure.stderr) {
      console.error(failure.stderr);
    }
  }
  process.exit(1);
}

console.log(`rbnx parser probes passed: ${probes.length}`);

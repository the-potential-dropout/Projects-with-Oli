// Builds the Rust simulation core (core/) to WebAssembly and generates the
// JavaScript bindings in src/core-pkg/. Requires:
//   rustup target add wasm32-unknown-unknown
//   cargo install wasm-bindgen-cli --version 0.2.130
import { execFileSync } from 'node:child_process';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const targetDir = process.env.CARGO_TARGET_DIR || join(root, 'core', 'target');
const run = (cmd, args) => execFileSync(cmd, args, { stdio: 'inherit', cwd: root, env: { ...process.env, CARGO_TARGET_DIR: targetDir } });

run('cargo', ['build', '--release', '--target', 'wasm32-unknown-unknown', '--manifest-path', join(root, 'core', 'Cargo.toml')]);
run('wasm-bindgen', [
  '--target', 'web',
  '--out-dir', join(root, 'src', 'core-pkg'),
  '--no-typescript',
  join(targetDir, 'wasm32-unknown-unknown', 'release', 'robot_core.wasm'),
]);
console.log('Rust core built -> src/core-pkg/');

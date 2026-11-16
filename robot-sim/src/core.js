// Loads the Rust simulation core (compiled to WebAssembly, see core/).
import init, { HexapodGait, World, solve_arm_2link, solve_legs } from './core-pkg/robot_core.js';
import wasmUrl from './core-pkg/robot_core_bg.wasm?url';

let ready = null;

export function loadCore() {
  if (!ready) ready = init({ module_or_path: wasmUrl });
  return ready;
}

export { HexapodGait, World, solve_arm_2link, solve_legs };

// Loads the Rust simulation core (compiled to WebAssembly, see core/).
import init, { Balance, HexapodGait, Terrain, World, solve_arm_2link, solve_legs } from './core-pkg/robot_core.js';
import wasmUrl from './core-pkg/robot_core_bg.wasm?url';

let ready = null;

export function loadCore() {
  if (!ready) ready = init({ module_or_path: wasmUrl });
  return ready;
}

export { Balance, HexapodGait, Terrain, World, solve_arm_2link, solve_legs };

// Real CAD meshes for the hexapod, exported from the Fusion design
// "Hexapod Mk1" (see cad/export_sim_meshes.py and cad/mesh/meshes.json).
//
// Frames (metres, +Y up):
//   body / qr_receiver / qr_adapter_example / sonar_example: body frame,
//     origin half way between the plates (= femur-axis height), +Z forward, +X robot-left
//   coxa:  origin on the coxa axis at femur-axis height, leg along +X
//   femur: origin on the femur axis, link along +X (tibia axis at x = 0.075)
//   tibia: origin on the tibia axis, shank along +X (foot contact at x = 0.120)
// If loading fails the hexapod falls back to simple built-in shapes.
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

const FILES = {
  body: new URL('../../cad/mesh/body.glb', import.meta.url).href,
  coxa: new URL('../../cad/mesh/coxa.glb', import.meta.url).href,
  femur: new URL('../../cad/mesh/femur.glb', import.meta.url).href,
  tibia: new URL('../../cad/mesh/tibia.glb', import.meta.url).href,
  qr_receiver: new URL('../../cad/mesh/qr_receiver.glb', import.meta.url).href,
  qr_adapter_example: new URL('../../cad/mesh/qr_adapter_example.glb', import.meta.url).href,
  sonar_example: new URL('../../cad/mesh/sonar_example.glb', import.meta.url).href,
};

// Printed parts get repainted in the robot's team colour (material names come from the CAD components)
const PRINTED = new Set(['Coxa', 'Femur', 'Tibia', 'QR_Receiver', 'PowerPanel', 'ControllerCarrier', 'FrontSensorBracket', 'BatteryTray']);

let meshes = null;
let loading = null;

/** Load every mesh once. Resolves to true if the CAD meshes are available. */
export function loadHexapodMeshes() {
  if (!loading) {
    const loader = new GLTFLoader();
    loading = Promise.all(
      Object.entries(FILES).map(([name, url]) =>
        loader.loadAsync(url).then((gltf) => {
          gltf.scene.traverse((o) => {
            if (o.isMesh) {
              o.castShadow = true;
              o.receiveShadow = true;
              if (o.material) {
                o.material.roughness = Math.max(o.material.roughness ?? 0.6, 0.45);
                o.material.metalness = Math.min(o.material.metalness ?? 0, 0.3);
              }
            }
          });
          return [name, gltf.scene];
        }),
      ),
    )
      .then((entries) => {
        meshes = Object.fromEntries(entries);
        return true;
      })
      .catch((e) => {
        console.warn('Hexapod CAD meshes failed to load, using simple shapes instead.', e);
        meshes = null;
        return false;
      });
  }
  return loading;
}

export const hasHexapodMeshes = () => meshes !== null;

/**
 * A copy of one mesh group. Printed parts use `teamMaterial`; `keep` optionally
 * filters which material names to include (e.g. only the adapter plate).
 */
export function hexapodMesh(name, teamMaterial, keep) {
  const src = meshes?.[name];
  if (!src) return null;
  const copy = src.clone(true);
  const drop = [];
  copy.traverse((o) => {
    if (!o.isMesh) return;
    const matName = o.material?.name ?? '';
    if (keep && !keep(matName)) drop.push(o);
    else if (teamMaterial && PRINTED.has(matName)) o.material = teamMaterial;
  });
  drop.forEach((o) => o.removeFromParent());
  return copy;
}

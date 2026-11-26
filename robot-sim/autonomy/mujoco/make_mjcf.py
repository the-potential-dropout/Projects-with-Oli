"""Generate the MuJoCo model (MJCF, primary) and a URDF (secondary, ROS 2 / RViz) of
Hexapod Mk1 from the CAD outputs, so the sim2real model always matches the robot.

    python make_mjcf.py

Inputs
  ../../cad/build_report.json   masses from the Fusion generator (printed parts, totals)
  ../../cad/mesh/*.glb          visual meshes (simulator frames)
Outputs (this folder)
  hexapod_mk1.xml               the robot (included by the scenes)
  scene_flat.xml, scene_terrain.xml
  meshes/*.stl                  visual meshes converted to the MuJoCo frames
  ../urdf/hexapod_mk1.urdf

Frames: MuJoCo world z up, x forward, y left.  Joint signs equal the simulator's
(coxa + = foot forward, femur + = up, tibia + = unfold, -90 deg = tibia vertical),
so qpos / ctrl are the sim's joint angles and the firmware's (firmware/hexapod_controller).
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.normpath(os.path.join(HERE, '..', '..'))
CAD = os.path.join(SIM, 'cad')
B = chr(92)


def L(p):
    p = os.path.abspath(p)
    return (B + B + '?' + B + p) if os.name == 'nt' and not p.startswith(B + B) else p


# ---------------------------------------------------------------- robot numbers (hexapod.config.js)
COXA, FEMUR, TIBIA = 0.045, 0.075, 0.120
STAND = 0.090
MOUNTS = [(-0.06, 0.11), (-0.08, 0.0), (-0.06, -0.11), (0.06, 0.11), (0.08, 0.0), (0.06, -0.11)]   # sim [x left, z fwd]
SPLAY = [35, 0, -35, 35, 0, -35]
NAMES = ['fr', 'mr', 'rr', 'fl', 'ml', 'rl']
LIM = {'coxa': (-45, 45), 'femur': (-10, 70), 'tibia': (-135, 30)}
FOOT_R = 0.011
IMU_POS = (-0.032, 0.010, -0.026)       # BNO085 in the IMU pod (Fusion (-10, -32, -26) mm)

# Feetech servo models (12 V).  Stall torque from the data sheets; kp chosen so 5 deg of
# position error gives the rated torque; armature = rotor inertia x gear ratio^2 (estimate).
SERVO = {
    'STS3215': dict(stall=30 * 0.0981, rated=10 * 0.0981, kp=11.0, kv=0.25, armature=0.008, speed=4.7),
    'STS3250': dict(stall=50 * 0.0981, rated=16 * 0.0981, kp=18.0, kv=0.35, armature=0.010, speed=4.4),
}
JOINT_SERVO = {'coxa': 'STS3215', 'femur': 'STS3250', 'tibia': 'STS3215'}


def masses():
    r = json.load(open(L(os.path.join(CAD, 'build_report.json')), encoding='utf-8'))
    each = {p['part']: p.get('est_g_each', 0.0) / 1000.0 for p in r['parts']}
    total = r['mass_g']['total'] / 1000.0
    coxa = each.get('CoxaMain', 0.03) + each.get('CoxaCap', 0.008) + 0.0745 + 0.006 + 0.008   # + STS3250, hub, leads
    femur = each.get('Femur', 0.03) + 0.069 + 0.006                                            # + tibia ST3215, leads
    tibia = each.get('Tibia', 0.03) + each.get('FootBoot_TPU', 0.005) + 0.0008 + 0.003         # + switch, lead
    body = total - 6 * (coxa + femur + tibia)
    return dict(total=total, body=body, coxa=coxa, femur=femur, tibia=tibia,
                brain=r['mass_g'].get('brain_module', 300) / 1000.0)


def sim_to_mj_body(v):          # sim body frame (x left, y up, z fwd) -> MuJoCo (x fwd, y left, z up)
    return np.array([v[2], v[0], v[1]])


def mount_pose(i):
    x, z = MOUNTS[i]
    splay = math.radians(SPLAY[i])
    yaw_sim = (math.pi - splay) if x < 0 else splay           # gaitMounts()
    pos = sim_to_mj_body((x, 0.0, z))
    d = np.array([math.sin(yaw_sim), math.cos(yaw_sim)])     # leg direction in MuJoCo (x fwd, y left)
    psi = math.atan2(d[1], d[0])
    return pos, psi


def quat_z(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def convert_meshes():
    import trimesh
    out = os.path.join(HERE, 'meshes')
    os.makedirs(L(out), exist_ok=True)
    P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)          # body: sim -> mj
    Q = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)         # legs: sim leg frame -> mj leg frame
    done = []
    for name, M in (('body', P), ('coxa', Q), ('femur', Q), ('tibia', Q), ('brain_module', P), ('qr_receiver', P)):
        src = os.path.join(CAD, 'mesh', name + '.glb')
        if not os.path.exists(L(src)):
            continue
        scene = trimesh.load(L(src), force='scene')
        mesh = scene.to_geometry() if hasattr(scene, 'to_geometry') else scene
        T = np.eye(4)
        T[:3, :3] = M
        mesh.apply_transform(T)
        mesh.export(L(os.path.join(out, name + '.stl')))
        done.append(name)
    return done


def leg_xml(i, m, have):
    pos, psi = mount_pose(i)
    n = NAMES[i]
    q = quat_z(psi)
    lim = {k: (math.radians(a), math.radians(b)) for k, (a, b) in LIM.items()}
    def vis(mesh):
        return (f'<geom type="mesh" mesh="{mesh}" class="visual"/>' if mesh in have else '')
    # coxa link: bracket + femur servo, centred ~ on the femur servo (case points down)
    return f"""
    <body name="{n}_coxa" pos="{pos[0]:.4f} {pos[1]:.4f} 0" quat="{q[0]:.6f} 0 0 {q[3]:.6f}">
      <joint name="{n}_coxa" class="coxa" range="{lim['coxa'][0]:.4f} {lim['coxa'][1]:.4f}"/>
      {vis('coxa')}
      <geom name="{n}_coxa_col" type="box" size="0.022 0.020 0.022" pos="0.035 0 -0.012" mass="{m['coxa']:.4f}" class="collision"/>
      <body name="{n}_femur" pos="{COXA} 0 0">
        <joint name="{n}_femur" class="femur" range="{lim['femur'][0]:.4f} {lim['femur'][1]:.4f}"/>
        {vis('femur')}
        <geom name="{n}_femur_col" type="box" size="0.030 0.026 0.016" pos="0.048 0 -0.002" mass="{m['femur']:.4f}" class="collision"/>
        <body name="{n}_tibia" pos="{FEMUR} 0 0">
          <joint name="{n}_tibia" class="tibia" range="{lim['tibia'][0]:.4f} {lim['tibia'][1]:.4f}"/>
          {vis('tibia')}
          <geom name="{n}_tibia_col" type="capsule" fromto="0.012 0 0 {TIBIA - FOOT_R - 0.012:.4f} 0 0" size="0.012" mass="{m['tibia']:.4f}" class="collision"/>
          <geom name="{n}_foot" type="sphere" pos="{TIBIA - FOOT_R:.4f} 0 0" size="{FOOT_R}" mass="0.004" class="foot"/>
          <site name="{n}_foot" pos="{TIBIA - FOOT_R:.4f} 0 0" size="{FOOT_R + 0.002}" type="sphere" rgba="0 1 0 0.2"/>
        </body>
      </body>
    </body>"""


def build():
    m = masses()
    have = convert_meshes()
    legs = ''.join(leg_xml(i, m, have) for i in range(6))
    act, sens = [], []
    for n in NAMES:
        for j in ('coxa', 'femur', 'tibia'):
            s = SERVO[JOINT_SERVO[j]]
            a, b = (math.radians(x) for x in LIM[j])
            act.append(f'<position name="{n}_{j}" joint="{n}_{j}" kp="{s["kp"]}" kv="{s["kv"]}" ctrlrange="{a:.4f} {b:.4f}" '
                       f'forcerange="{-s["stall"]:.3f} {s["stall"]:.3f}"/>')
            sens.append(f'<jointpos name="{n}_{j}_pos" joint="{n}_{j}"/><jointvel name="{n}_{j}_vel" joint="{n}_{j}"/>')
        sens.append(f'<touch name="{n}_contact" site="{n}_foot"/>')
    body_vis = '<geom type="mesh" mesh="body" class="visual"/>' if 'body' in have else ''
    brain_vis = '<geom type="mesh" mesh="brain_module" class="visual" rgba="0.5 0.55 0.6 1"/>' if 'brain_module' in have else ''
    meshes = ''.join(f'<mesh name="{k}" file="meshes/{k}.stl"/>' for k in have)
    # stand keyframe from the firmware/sim IK: foot 0.1 out, 0.09 down
    hz = math.hypot(0.1 - COXA, STAND)
    cos_a = (FEMUR ** 2 + hz ** 2 - TIBIA ** 2) / (2 * FEMUR * hz)
    femur = math.atan2(-STAND, 0.1 - COXA) + math.acos(cos_a)
    tibia = math.acos((FEMUR ** 2 + TIBIA ** 2 - hz ** 2) / (2 * FEMUR * TIBIA)) - math.pi
    stand = ' '.join(f'0 {femur:.4f} {tibia:.4f}' for _ in range(6))
    xml = f"""<mujoco model="hexapod_mk1">
  <!-- Generated by make_mjcf.py from cad/build_report.json and cad/mesh - do not edit by hand.
       Robot mass {m['total']:.3f} kg (body {m['body']:.3f}, per leg coxa {m['coxa']:.3f} / femur {m['femur']:.3f} / tibia {m['tibia']:.3f}).
       No passive joints or linkages: the Mk1 leg is a serial 3-DOF chain (docs/KINEMATICS.md). -->
  <compiler angle="radian" autolimits="true" inertiafromgeom="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <default>
    <joint type="hinge" damping="0.05" frictionloss="0.03" armature="0.008"/>
    <default class="coxa"><joint axis="0 0 -1"/></default>
    <default class="femur"><joint axis="0 -1 0" armature="0.010"/></default>
    <default class="tibia"><joint axis="0 -1 0"/></default>
    <default class="visual"><geom contype="0" conaffinity="0" group="2" mass="0" rgba="0.85 0.35 0.12 1"/></default>
    <default class="collision"><geom group="3" rgba="0.2 0.2 0.25 0.4" condim="3" friction="0.6 0.005 0.0001"/></default>
    <default class="foot"><geom group="3" condim="3" friction="0.9 0.005 0.0001" solref="0.004 1" rgba="0.1 0.1 0.1 1"/></default>
  </default>
  <asset>{meshes}</asset>
  <worldbody>
    <body name="torso" pos="0 0 {STAND + 0.002:.3f}">
      <freejoint name="root"/>
      {body_vis}
      <geom name="torso_col" type="box" size="0.130 0.065 0.020" pos="0 0 -0.0014" mass="{m['body'] - 0.305:.4f}" class="collision"/>
      <geom name="battery" type="box" size="0.069 0.0215 0.012" pos="-0.043 0 -0.0023" mass="0.305" contype="0" conaffinity="0" group="3"/>
      {brain_vis}
      <site name="imu" pos="{IMU_POS[0]} {IMU_POS[1]} {IMU_POS[2]}" size="0.006"/>
      <site name="top_mount" pos="-0.01 0 0.0318" size="0.006"/>
      <body name="payload" pos="-0.012 0 0.0318">
        <!-- top-mount attachment: the brain module by default; the training env randomises 0 - 0.4 kg -->
        <geom name="payload" type="box" size="0.050 0.051 0.019" pos="0 0 0.019" mass="{m['brain']:.4f}" class="collision"/>
      </body>{legs}
    </body>
  </worldbody>
  <actuator>
    {chr(10).join('    ' + a for a in act).strip()}
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    {chr(10).join('    ' + s for s in sens).strip()}
  </sensor>
  <keyframe>
    <key name="stand" qpos="0 0 {STAND + 0.002:.3f} 1 0 0 0 {stand}" ctrl="{stand}"/>
  </keyframe>
</mujoco>
"""
    open(L(os.path.join(HERE, 'hexapod_mk1.xml')), 'w', encoding='utf-8').write(xml)
    floor = """<mujoco model="hexapod_mk1 {name}">
  <include file="hexapod_mk1.xml"/>
  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.25 0.3" rgb2="0.15 0.18 0.22" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>{extra_asset}
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    {ground}
  </worldbody>
</mujoco>
"""
    open(L(os.path.join(HERE, 'scene_flat.xml')), 'w', encoding='utf-8').write(floor.format(
        name='flat', extra_asset='', ground='<geom name="floor" type="plane" size="4 4 0.1" material="grid" friction="0.9 0.005 0.0001"/>'))
    # heightfield: 6 x 6 m, up to 6 cm bumps; elevation is filled in by the env (hexapod_env.py)
    open(L(os.path.join(HERE, 'scene_terrain.xml')), 'w', encoding='utf-8').write(floor.format(
        name='terrain', extra_asset='\n    <hfield name="terrain" nrow="200" ncol="200" size="3 3 0.06 0.02"/>',
        ground='<geom name="terrain" type="hfield" hfield="terrain" material="grid" friction="0.9 0.005 0.0001"/>'))
    write_urdf(m)
    return m, have


def write_urdf(m):
    """Minimal URDF for ROS 2 (robot_state_publisher / RViz / Nav2 footprint)."""
    def inertial(mass, sx, sy, sz, xyz):
        ixx, iyy, izz = mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12, mass * (sx * sx + sy * sy) / 12
        return (f'<inertial><origin xyz="{xyz}"/><mass value="{mass:.4f}"/>'
                f'<inertia ixx="{ixx:.3e}" iyy="{iyy:.3e}" izz="{izz:.3e}" ixy="0" ixz="0" iyz="0"/></inertial>')
    out = ['<?xml version="1.0"?>', '<robot name="hexapod_mk1">',
           '  <link name="base_link">' + inertial(m['body'], 0.26, 0.13, 0.04, '0 0 0') +
           '<visual><geometry><mesh filename="package://hexapod_mk1/meshes/body.stl"/></geometry></visual></link>',
           '  <link name="imu_link"/>',
           f'  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="{IMU_POS[0]} {IMU_POS[1]} {IMU_POS[2]}"/></joint>']
    for i, n in enumerate(NAMES):
        pos, psi = mount_pose(i)
        for j, (parent, child, xyz, axis, mass, mesh) in enumerate((
                ('base_link', n + '_coxa', f'{pos[0]:.4f} {pos[1]:.4f} 0', '0 0 -1', m['coxa'], 'coxa'),
                (n + '_coxa', n + '_femur', f'{COXA} 0 0', '0 -1 0', m['femur'], 'femur'),
                (n + '_femur', n + '_tibia', f'{FEMUR} 0 0', '0 -1 0', m['tibia'], 'tibia'))):
            jn = child
            lo, hi = (math.radians(x) for x in LIM[jn.split('_')[1]])
            rpy = f'0 0 {psi:.5f}' if j == 0 else '0 0 0'
            out.append(f'  <link name="{child}">' + inertial(mass, 0.06, 0.04, 0.04, '0.03 0 0') +
                       f'<visual><geometry><mesh filename="package://hexapod_mk1/meshes/{mesh}.stl"/></geometry></visual></link>')
            out.append(f'  <joint name="{jn}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
                       f'<origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/>'
                       f'<limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{SERVO[JOINT_SERVO[jn.split("_")[1]]]["stall"]:.2f}" '
                       f'velocity="{SERVO[JOINT_SERVO[jn.split("_")[1]]]["speed"]}"/></joint>')
        out.append(f'  <link name="{n}_foot"/><joint name="{n}_foot_joint" type="fixed"><parent link="{n}_tibia"/>'
                   f'<child link="{n}_foot"/><origin xyz="{TIBIA} 0 0"/></joint>')
    out.append('</robot>')
    d = os.path.join(HERE, '..', 'urdf')
    os.makedirs(L(d), exist_ok=True)
    open(L(os.path.join(d, 'hexapod_mk1.urdf')), 'w', encoding='utf-8').write('\n'.join(out) + '\n')


if __name__ == '__main__':
    m, have = build()
    print('masses (kg):', {k: round(v, 3) for k, v in m.items()})
    print('meshes:', have)

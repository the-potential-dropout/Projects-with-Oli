// Digital twin panel: shows the servo angles the simulator is commanding, and
// (in Chrome/Edge on a computer) can stream them over USB to the real robot's
// microcontroller using Web Serial. Protocol, one line per frame at 25 Hz:
//   S,<a0>,<a1>,...,<aN>\n       angles in whole degrees 0-180
// Order: leg 0 coxa, femur, tibia, leg 1 ..., leg 5 tibia, then attachment
// servos (turret pitch, or arm base/shoulder/elbow/wrist/gripper).
// See docs/BUILD.md for the Arduino/ESP32 side.
//
// Rev B2 adds command mode (docs/CONTROL.md §6): instead of every servo angle the
// sim sends what it wants, C,vx,vz,yaw,roll,pitch,height,gait,flags at 10 Hz, and
// the robot runs its own gait, balance and terrain. A,1 arms it when no radio is
// linked. The robot answers with B,<mV>,<mA>,<degC>,<state> and
// T,roll,pitch,yaw,feet,margin_mm,mode,source,lq lines, shown in the panel.
import { GAITS } from './robots/hexapod.control.js';

const LEG_NAMES = ['Front R', 'Mid R', 'Rear R', 'Front L', 'Mid L', 'Rear L'];

export class TwinPanel {
  constructor(root) {
    this.root = root;
    this.table = root.querySelector('#twin-table');
    this.serialBox = root.querySelector('#twin-serial');
    this.port = null;
    this.writer = null;
    this.sendTimer = 0;
    this.renderTimer = 0;
    this.linkMode = 'joint'; // 'joint' (S lines) or 'command' (C lines)
    this.robotLines = { B: null, T: null };
    this.lastCmd = {};
    root.querySelector('#twin-close').addEventListener('click', () => this.toggle(false));
    this.#serialUi();
  }

  get open() {
    return !this.root.classList.contains('hidden');
  }

  toggle(force) {
    const show = force ?? !this.open;
    this.root.classList.toggle('hidden', !show);
  }

  #serialUi() {
    const box = this.serialBox;
    box.innerHTML = '';
    if (!('serial' in navigator)) {
      box.innerHTML = '<span class="hint">USB link to the real robot needs Chrome or Edge on a computer.</span>';
      return;
    }
    const btn = document.createElement('button');
    const state = document.createElement('span');
    state.className = 'hint';
    const refresh = () => {
      btn.textContent = this.writer ? 'Disconnect robot' : 'Connect robot (USB)';
      state.textContent = this.writer ? 'Streaming at 25 Hz' : 'Not connected';
    };
    btn.addEventListener('click', async () => {
      try {
        if (this.writer) await this.disconnect();
        else await this.connect();
      } catch (e) {
        state.textContent = `Error: ${e.message}`;
        return;
      }
      refresh();
    });
    const mode = document.createElement('select');
    mode.innerHTML = '<option value="joint">Joint bridge: sim drives every servo (S)</option><option value="command">Command mode: robot balances itself (C)</option>';
    mode.value = this.linkMode;
    mode.addEventListener('change', () => (this.linkMode = mode.value));
    this.robotBox = document.createElement('div');
    this.robotBox.className = 'hint';
    box.append(btn, state, mode, this.robotBox);
    refresh();
  }

  async connect() {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200 });
    this.writer = this.port.writable.getWriter();
    this.armed = false;
    this.#readLoop();
  }

  async #readLoop() {
    try {
      const reader = this.port.readable.pipeThrough(new TextDecoderStream()).getReader();
      this.reader = reader;
      let buf = '';
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += value;
        let i;
        while ((i = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, i).trim();
          buf = buf.slice(i + 1);
          if (line[1] === ',' && (line[0] === 'B' || line[0] === 'T')) this.robotLines[line[0]] = line.slice(2).split(',');
        }
      }
    } catch {
      /* port closed */
    }
  }

  async disconnect() {
    try {
      await this.reader?.cancel().catch(() => {});
      this.writer?.releaseLock();
      await this.port?.close();
    } finally {
      this.writer = null;
      this.port = null;
    }
  }

  update(dt, robot, cmd) {
    if (!robot) return;
    if (cmd) this.lastCmd = cmd;
    this.sendTimer -= dt;
    if (this.writer && this.sendTimer <= 0) {
      let line;
      if (this.linkMode === 'command' && robot.ctl) {
        this.sendTimer = 0.1;
        const c = robot.ctl;
        const v = robot.vel;
        const flags = (c.terrainMode ? 1 : 0) | (this.lastCmd.fire ? 2 : 0) | (this.lastCmd.sprint ? 4 : 0);
        const d = (r) => ((r * 180) / Math.PI).toFixed(1);
        line = `${this.armed ? '' : 'A,1\n'}C,${v.x.toFixed(3)},${v.z.toFixed(3)},${v.w.toFixed(3)},${d(c.req.roll)},${d(c.req.pitch)},${c.height.toFixed(3)},${c.gaitReq},${flags}\n`;
        this.armed = true;
      } else {
        this.sendTimer = 0.04;
        line = `S,${robot.servoFrame().join(',')}\n`;
      }
      this.writer.write(new TextEncoder().encode(line)).catch(() => this.disconnect().then(() => this.#serialUi()));
    }
    if (!this.open) return;
    this.renderTimer -= dt;
    if (this.renderTimer > 0) return;
    this.renderTimer = 0.1;
    const f = robot.servoFrame();
    let html = '<tr><th>Leg</th><th>Coxa</th><th>Femur</th><th>Tibia</th></tr>';
    for (let i = 0; i < 6; i++) {
      html += `<tr><td>${LEG_NAMES[i]}</td><td>${f[i * 3]}°</td><td>${f[i * 3 + 1]}°</td><td>${f[i * 3 + 2]}°</td></tr>`;
    }
    const extra = f.slice(18);
    if (extra.length) {
      const names = robot.attachments.top?.id === 'arm' ? ['Arm base', 'Shoulder', 'Elbow', 'Wrist', 'Gripper'] : ['Turret pitch'];
      extra.forEach((v, i) => {
        html += `<tr><td>${names[i] ?? 'Aux ' + i}</td><td colspan="3">${v}°</td></tr>`;
      });
    }
    html += `<tr><td>Health</td><td colspan="3">${Math.round(robot.hp)} / ${robot.maxHp}</td></tr>`;
    const c = robot.ctl;
    if (c) {
      const deg = (r) => `${((r * 180) / Math.PI).toFixed(1)}°`;
      html += `<tr><td>Sim IMU</td><td colspan="3">roll ${deg(c.imu.roll)} · pitch ${deg(c.imu.pitch)}</td></tr>`;
      html += `<tr><td>Gait</td><td colspan="3">${GAITS[robot.gait.current_gait()]} · ${c.mode}${c.terrainMode ? ' · terrain' : ''}</td></tr>`;
      html += `<tr><td>Sim line</td><td colspan="3"><code>${c.telemetry(robot.yaw)}</code></td></tr>`;
    }
    if (this.robotBox) {
      const B = this.robotLines.B;
      const T = this.robotLines.T;
      this.robotBox.innerHTML = this.writer
        ? `Robot: ${B ? `${(B[0] / 1000).toFixed(2)} V, ${(B[1] / 1000).toFixed(1)} A, ${B[2]} °C, ${B[3]}` : 'no battery line yet'}${T ? `<br>roll ${T[0]}°, pitch ${T[1]}°, feet ${(+T[3]).toString(2).padStart(6, '0')}, margin ${T[4]} mm` : ''}`
        : '';
    }
    this.table.innerHTML = html;
  }
}

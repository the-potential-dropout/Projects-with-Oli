// Digital twin panel: shows the servo angles the simulator is commanding, and
// (in Chrome/Edge on a computer) can stream them over USB to the real robot's
// microcontroller using Web Serial. Protocol, one line per frame at 25 Hz:
//   S,<a0>,<a1>,...,<aN>\n       angles in whole degrees 0-180
// Order: leg 0 coxa, femur, tibia, leg 1 ..., leg 5 tibia, then attachment
// servos (turret pitch, or arm base/shoulder/elbow/wrist/gripper).
// See docs/BUILD.md for the Arduino/ESP32 side.

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
    box.append(btn, state);
    refresh();
  }

  async connect() {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200 });
    this.writer = this.port.writable.getWriter();
  }

  async disconnect() {
    try {
      this.writer?.releaseLock();
      await this.port?.close();
    } finally {
      this.writer = null;
      this.port = null;
    }
  }

  update(dt, robot) {
    if (!robot) return;
    this.sendTimer -= dt;
    if (this.writer && this.sendTimer <= 0) {
      this.sendTimer = 0.04;
      const line = `S,${robot.servoFrame().join(',')}\n`;
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
    this.table.innerHTML = html;
  }
}

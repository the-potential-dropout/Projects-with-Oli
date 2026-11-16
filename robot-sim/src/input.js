// Player input: keyboard + mouse, gamepads (Xbox / PlayStation / Switch Pro via
// the browser Gamepad API) and on-screen touch joysticks. All of them are merged
// into one command object that the robot understands:
//   { moveX, moveZ, turn, aim, sprint, fire, firePressed, alt, altPressed }
// moveX: +1 = strafe left, moveZ: +1 = forward, turn: +1 = turn left, aim: +1 = aim up

const DEAD = 0.15;
const dz = (v) => (Math.abs(v) < DEAD ? 0 : (v - Math.sign(v) * DEAD) / (1 - DEAD));

export class Input {
  constructor(canvas, touchRoot) {
    this.keys = new Set();
    this.mouse = { dx: 0, dy: 0, fire: false, alt: false, locked: false };
    this.touch = { move: { x: 0, y: 0 }, look: { x: 0, y: 0 }, fire: false, alt: false };
    this.prev = { fire: false, alt: false };
    this.lastDevice = 'keyboard';
    this.enabled = true;

    addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
      this.keys.add(e.code);
      this.lastDevice = 'keyboard';
      if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) e.preventDefault();
    });
    addEventListener('keyup', (e) => this.keys.delete(e.code));
    addEventListener('blur', () => this.keys.clear());

    // Mouse: click the 3D view to capture the mouse (desktop). Move = turn/aim.
    canvas.addEventListener('mousedown', (e) => {
      if (!this.enabled) return;
      if (!this.mouse.locked && canvas.requestPointerLock && !matchMedia('(pointer: coarse)').matches) {
        try {
          const p = canvas.requestPointerLock();
          if (p && p.catch) p.catch(() => {});
        } catch {
          /* Safari may refuse; mouse buttons still work */
        }
      }
      if (e.button === 0) this.mouse.fire = true;
      if (e.button === 2) this.mouse.alt = true;
      this.lastDevice = 'keyboard';
    });
    addEventListener('mouseup', (e) => {
      if (e.button === 0) this.mouse.fire = false;
      if (e.button === 2) this.mouse.alt = false;
    });
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    document.addEventListener('pointerlockchange', () => {
      this.mouse.locked = document.pointerLockElement === canvas;
    });
    addEventListener('mousemove', (e) => {
      if (!this.mouse.locked) return;
      this.mouse.dx += e.movementX || 0;
      this.mouse.dy += e.movementY || 0;
    });

    this.#setupTouch(touchRoot);
  }

  #setupTouch(root) {
    const stick = (el, out) => {
      const knob = el.querySelector('.knob');
      let id = null;
      let cx = 0;
      let cy = 0;
      const R = () => el.clientWidth / 2;
      const move = (x, y) => {
        let dx = (x - cx) / R();
        let dy = (y - cy) / R();
        const l = Math.hypot(dx, dy);
        if (l > 1) {
          dx /= l;
          dy /= l;
        }
        out.x = dx;
        out.y = -dy;
        knob.style.transform = `translate(${dx * R() * 0.6}px, ${dy * R() * 0.6}px)`;
      };
      el.addEventListener('pointerdown', (e) => {
        id = e.pointerId;
        const r = el.getBoundingClientRect();
        cx = r.left + r.width / 2;
        cy = r.top + r.height / 2;
        el.setPointerCapture?.(id);
        move(e.clientX, e.clientY);
        this.lastDevice = 'touch';
        e.preventDefault();
      });
      el.addEventListener('pointermove', (e) => {
        if (e.pointerId === id) move(e.clientX, e.clientY);
      });
      const end = (e) => {
        if (e.pointerId !== id) return;
        id = null;
        out.x = out.y = 0;
        knob.style.transform = '';
      };
      el.addEventListener('pointerup', end);
      el.addEventListener('pointercancel', end);
    };
    stick(root.querySelector('#stick-move'), this.touch.move);
    stick(root.querySelector('#stick-look'), this.touch.look);
    const button = (el, key) => {
      const on = (e) => {
        this.touch[key] = true;
        el.classList.add('down');
        this.lastDevice = 'touch';
        e.preventDefault();
      };
      const off = () => {
        this.touch[key] = false;
        el.classList.remove('down');
      };
      el.addEventListener('pointerdown', on);
      el.addEventListener('pointerup', off);
      el.addEventListener('pointercancel', off);
      el.addEventListener('pointerleave', off);
    };
    button(root.querySelector('#btn-fire'), 'fire');
    button(root.querySelector('#btn-alt'), 'alt');
  }

  #gamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const p of pads) {
      if (!p || !p.connected) continue;
      const b = (i) => (p.buttons[i] ? p.buttons[i].pressed || p.buttons[i].value > 0.4 : false);
      const pad = {
        moveX: -dz(p.axes[0] || 0),
        moveZ: -dz(p.axes[1] || 0),
        turn: -dz(p.axes[2] || 0),
        aim: -dz(p.axes[3] || 0),
        fire: b(7) || b(0),
        alt: b(6) || b(5) || b(2),
        sprint: b(10) || b(4),
        pause: b(9),
      };
      const active = Object.values(pad).some((v) => v === true || (typeof v === 'number' && v !== 0));
      if (active) this.lastDevice = 'gamepad';
      return pad;
    }
    return null;
  }

  /** Read and merge every input source. Call once per frame. */
  read(dt) {
    const k = (c) => this.keys.has(c);
    const cmd = { moveX: 0, moveZ: 0, turn: 0, aim: 0, sprint: false, fire: false, alt: false, pause: false };
    if (!this.enabled) return { ...cmd, firePressed: false, altPressed: false };

    cmd.moveZ += (k('KeyW') ? 1 : 0) - (k('KeyS') ? 1 : 0);
    cmd.moveX += (k('KeyA') ? 1 : 0) - (k('KeyD') ? 1 : 0);
    cmd.turn += (k('KeyQ') || k('ArrowLeft') ? 1 : 0) - (k('KeyE') || k('ArrowRight') ? 1 : 0);
    cmd.aim += (k('ArrowUp') || k('KeyR') ? 1 : 0) - (k('ArrowDown') || k('KeyF') ? 1 : 0);
    cmd.sprint = k('ShiftLeft') || k('ShiftRight');
    cmd.fire = k('Space') || this.mouse.fire;
    cmd.alt = k('KeyG') || k('KeyX') || this.mouse.alt;

    // Mouse look (only while pointer is locked)
    if (dt > 0) {
      cmd.turn += -this.mouse.dx * 0.06;
      cmd.aim += -this.mouse.dy * 0.08;
    }
    this.mouse.dx = this.mouse.dy = 0;

    // Touch sticks
    const t = this.touch;
    cmd.moveX += -t.move.x;
    cmd.moveZ += t.move.y;
    cmd.turn += -t.look.x;
    cmd.aim += t.look.y;
    if (Math.hypot(t.move.x, t.move.y) > 0.92) cmd.sprint = true;
    cmd.fire ||= t.fire;
    cmd.alt ||= t.alt;

    const pad = this.#gamepad();
    if (pad) {
      cmd.moveX += pad.moveX;
      cmd.moveZ += pad.moveZ;
      cmd.turn += pad.turn;
      cmd.aim += pad.aim;
      cmd.fire ||= pad.fire;
      cmd.alt ||= pad.alt;
      cmd.sprint ||= pad.sprint;
      cmd.pause = pad.pause;
    }

    const clamp = (v) => Math.max(-1, Math.min(1, v));
    cmd.moveX = clamp(cmd.moveX);
    cmd.moveZ = clamp(cmd.moveZ);
    cmd.turn = clamp(cmd.turn);
    cmd.aim = clamp(cmd.aim);
    cmd.firePressed = cmd.fire && !this.prev.fire;
    cmd.altPressed = cmd.alt && !this.prev.alt;
    this.prev.fire = cmd.fire;
    this.prev.alt = cmd.alt;
    return cmd;
  }

  releasePointer() {
    if (document.pointerLockElement) document.exitPointerLock?.();
  }
}

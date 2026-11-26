// Player input: keyboard + mouse, gamepads (Xbox / PlayStation / Switch Pro via
// the browser Gamepad API) and on-screen touch joysticks. All of them are merged
// into one command object that the robot understands:
//   { moveX, moveZ, turn, aim, sprint, fire, firePressed, alt, altPressed }
// moveX: +1 = strafe left, moveZ: +1 = forward, turn: +1 = turn left, aim: +1 = aim up
//
// Plus the robot's radio switches (radio/README.md), as one-shot edges:
//   gait (0 tripod / 1 ripple / 2 wave, SB), gaitStep, toggleTerrain (SD),
//   cycleMode (SC: manual / pose / auto), toggleBalance, heightRate (throttle).
//
// Layout 'game' is the usual video-game mapping. Layout 'radio' matches the
// robot's RadioMaster in Mode 2 (AETR): right stick walks and strafes, left
// stick turns (left/right) and sets the body height (up/down).

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
    this.layout = 'game';
    this.edges = new Set(); // keys pressed since the last read()
    this.padPrev = [];
    this.touchEdges = new Set();

    addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
      this.keys.add(e.code);
      if (!e.repeat) this.edges.add(e.code);
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
    // radio-switch buttons (one tap = one switch flick)
    root.querySelectorAll('[data-switch]').forEach((el) => {
      el.addEventListener('pointerdown', (e) => {
        this.touchEdges.add(el.dataset.switch);
        this.lastDevice = 'touch';
        e.preventDefault();
      });
    });
  }

  #gamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const p of pads) {
      if (!p || !p.connected) continue;
      const b = (i) => (p.buttons[i] ? p.buttons[i].pressed || p.buttons[i].value > 0.4 : false);
      const edge = (i) => b(i) && !this.padPrev[i];
      const radio = this.layout === 'radio';
      const ax = (i) => dz(p.axes[i] || 0);
      const pad = radio
        ? {
            // Mode 2 radio: right stick = walk / strafe, left = turn / body height
            moveX: -ax(2),
            moveZ: -ax(3),
            turn: -ax(0),
            heightRate: -ax(1),
            aim: (b(12) ? 1 : 0) - (b(13) ? 1 : 0),
          }
        : { moveX: -ax(0), moveZ: -ax(1), turn: -ax(2), aim: -ax(3), heightRate: (b(12) ? 1 : 0) - (b(13) ? 1 : 0) };
      Object.assign(pad, {
        fire: b(7) || b(0),
        alt: b(6) || b(5) || b(2),
        sprint: b(10) || b(4),
        pause: b(9),
        gaitStep: (edge(15) ? 1 : 0) - (edge(14) ? 1 : 0),
        cycleMode: edge(3),
        toggleTerrain: edge(8),
        toggleBalance: edge(11),
      });
      this.padPrev = p.buttons.map((x) => x.pressed);
      const active = Object.values(pad).some((v) => v === true || (typeof v === 'number' && v !== 0));
      pad.active = active;
      if (active) this.lastDevice = 'gamepad';
      return pad;
    }
    return null;
  }

  /** Read and merge every input source. Call once per frame. */
  read(dt) {
    const k = (c) => this.keys.has(c);
    const cmd = { moveX: 0, moveZ: 0, turn: 0, aim: 0, sprint: false, fire: false, alt: false, pause: false, heightRate: 0 };
    const edges = this.edges;
    this.edges = new Set();
    const tEdges = this.touchEdges;
    this.touchEdges = new Set();
    if (!this.enabled) return { ...cmd, firePressed: false, altPressed: false };

    // radio switches on the keyboard: 1 2 3 gait, V terrain, B balance, M mode, [ ] height
    const e = (c) => edges.has(c);
    if (e('Digit1')) cmd.gait = 0;
    if (e('Digit2')) cmd.gait = 1;
    if (e('Digit3')) cmd.gait = 2;
    cmd.toggleTerrain = e('KeyV') || tEdges.has('terrain');
    cmd.toggleBalance = e('KeyB');
    cmd.cycleMode = e('KeyM') || tEdges.has('mode');
    if (tEdges.has('gait')) cmd.gaitStep = 1;
    cmd.heightRate += (k('BracketRight') || k('Equal') ? 1 : 0) - (k('BracketLeft') || k('Minus') ? 1 : 0);

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
    if (this.layout === 'radio') {
      // left stick = turn / body height, right stick = walk / strafe (Mode 2 radio)
      cmd.turn += -t.move.x;
      cmd.heightRate += t.move.y;
      cmd.moveX += -t.look.x;
      cmd.moveZ += t.look.y;
      if (Math.hypot(t.look.x, t.look.y) > 0.92) cmd.sprint = true;
    } else {
      cmd.moveX += -t.move.x;
      cmd.moveZ += t.move.y;
      cmd.turn += -t.look.x;
      cmd.aim += t.look.y;
      if (Math.hypot(t.move.x, t.move.y) > 0.92) cmd.sprint = true;
    }
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
      cmd.heightRate += pad.heightRate || 0;
      if (pad.gaitStep) cmd.gaitStep = pad.gaitStep;
      cmd.cycleMode ||= pad.cycleMode;
      cmd.toggleTerrain ||= pad.toggleTerrain;
      cmd.toggleBalance ||= pad.toggleBalance;
    }

    const clamp = (v) => Math.max(-1, Math.min(1, v));
    cmd.moveX = clamp(cmd.moveX);
    cmd.moveZ = clamp(cmd.moveZ);
    cmd.turn = clamp(cmd.turn);
    cmd.aim = clamp(cmd.aim);
    cmd.heightRate = clamp(cmd.heightRate);
    // any stick input (as on the real robot: it takes control back from auto)
    cmd.sticks = Math.abs(cmd.moveX) + Math.abs(cmd.moveZ) + Math.abs(cmd.turn) > 0.25;
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

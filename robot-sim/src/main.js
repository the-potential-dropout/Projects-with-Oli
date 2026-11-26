import './style.css';
import { loadCore } from './core.js';
import { loadHexapodMeshes } from './robots/hexapod.meshes.js';
import { Game } from './game.js';
import { ROBOTS } from './robots/index.js';
import { ATTACHMENTS, attachmentsForSlot } from './attachments.js';

const $ = (s) => document.querySelector(s);

const STORE_KEY = 'robot-sim-settings-v1';
const defaults = {
  robot: 'hexapod',
  mode: 'battle',
  you: { top: 'laser', front: 'sonar' },
  bot: { top: 'random', front: 'random' },
  botLevel: 'normal',
  ground: 'flat',
  layout: 'game',
};

function loadSettings() {
  try {
    const s = JSON.parse(localStorage.getItem(STORE_KEY));
    return s ? { ...defaults, ...s, you: { ...defaults.you, ...s.you }, bot: { ...defaults.bot, ...s.bot } } : structuredCloneSafe(defaults);
  } catch {
    return structuredCloneSafe(defaults);
  }
}

function saveSettings(s) {
  try {
    localStorage.setItem(STORE_KEY, JSON.stringify(s));
  } catch {
    /* private mode etc. – settings just won't be remembered */
  }
}

function structuredCloneSafe(o) {
  return JSON.parse(JSON.stringify(o));
}

function fillSelect(sel, slot, value, withRandom) {
  sel.innerHTML = '';
  if (withRandom) sel.append(new Option('Random', 'random'));
  for (const a of attachmentsForSlot(slot)) sel.append(new Option(a.name, a.id));
  sel.value = value;
  if (sel.value !== value) sel.value = withRandom ? 'random' : 'none';
}

/** Resolve "random" picks for the bot. */
function resolveBotLoadout(bot) {
  const pick = (slot) => {
    const opts = attachmentsForSlot(slot).filter((a) => a.id !== 'none');
    return opts[Math.floor(Math.random() * opts.length)].id;
  };
  return {
    top: bot.top === 'random' ? pick('top') : bot.top,
    front: bot.front === 'random' ? pick('front') : bot.front,
  };
}

async function boot() {
  try {
    // Rust core is required; the CAD meshes fall back to simple shapes if they fail
    await Promise.all([loadCore(), loadHexapodMeshes()]);
  } catch (e) {
    $('#loading p').textContent = `Could not start the simulator core (${e.message}). Try a newer browser.`;
    return;
  }
  const game = new Game();
  const settings = loadSettings();

  // Robot cards
  const cards = $('#robot-cards');
  for (const r of ROBOTS) {
    const b = document.createElement('button');
    b.className = `card${r.id === settings.robot ? ' on' : ''}`;
    b.disabled = !r.available;
    b.innerHTML = `<strong>${r.name}</strong><span>${r.blurb}</span>`;
    b.addEventListener('click', () => {
      settings.robot = r.id;
      cards.querySelectorAll('.card').forEach((c) => c.classList.toggle('on', c === b));
    });
    cards.append(b);
  }

  // Mode
  const modeButtons = document.querySelectorAll('#mode-seg button');
  const setMode = (m) => {
    settings.mode = m;
    modeButtons.forEach((b) => b.classList.toggle('on', b.dataset.mode === m));
    $('#bot-config').style.opacity = m === 'battle' ? 1 : 0.4;
    $('#bot-config').inert = m !== 'battle';
  };
  modeButtons.forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
  setMode(settings.mode);

  // Ground and control layout
  const seg = (id, key, onChange) => {
    const buttons = document.querySelectorAll(`#${id} button`);
    const set = (v) => {
      settings[key] = v;
      buttons.forEach((b) => b.classList.toggle('on', b.dataset[key] === v));
    };
    buttons.forEach((b) =>
      b.addEventListener('click', () => {
        set(b.dataset[key]);
        saveSettings(settings);
        onChange?.();
      }),
    );
    set(settings[key]);
  };
  seg('ground-seg', 'ground', () => game.showGarage(settings));
  seg('layout-seg', 'layout');

  // Attachments
  const desc = (id) => ATTACHMENTS[id]?.desc ?? 'The bot picks something at random.';
  const youTop = $('#you-top');
  const youFront = $('#you-front');
  fillSelect(youTop, 'top', settings.you.top);
  fillSelect(youFront, 'front', settings.you.front);
  fillSelect($('#bot-top'), 'top', settings.bot.top, true);
  fillSelect($('#bot-front'), 'front', settings.bot.front, true);
  $('#bot-level').value = settings.botLevel;
  const syncDesc = () => {
    $('#you-top-desc').textContent = desc(youTop.value);
    $('#you-front-desc').textContent = desc(youFront.value);
  };
  const onLoadout = () => {
    settings.you = { top: youTop.value, front: youFront.value };
    syncDesc();
    game.updateGarageLoadout(settings);
    saveSettings(settings);
  };
  youTop.addEventListener('change', onLoadout);
  youFront.addEventListener('change', onLoadout);
  $('#bot-top').addEventListener('change', (e) => (settings.bot.top = e.target.value));
  $('#bot-front').addEventListener('change', (e) => (settings.bot.front = e.target.value));
  $('#bot-level').addEventListener('change', (e) => (settings.botLevel = e.target.value));
  syncDesc();

  let matchSettings = null;
  const start = () => {
    saveSettings(settings);
    matchSettings = { ...settings, bot: resolveBotLoadout(settings.bot) };
    $('#menu').classList.add('hidden');
    game.startMatch(matchSettings);
  };
  const toGarage = () => {
    $('#pause').classList.add('hidden');
    game.input.enabled = true;
    game.showGarage(settings);
    $('#menu').classList.remove('hidden');
  };
  game.onGarage = toGarage;
  $('#start').addEventListener('click', start);

  // Pause menu
  $('#btn-pause').addEventListener('click', () => game.pause(true));
  $('#resume').addEventListener('click', () => game.pause(false));
  $('#restart').addEventListener('click', () => {
    game.pause(false);
    matchSettings = { ...settings, bot: resolveBotLoadout(settings.bot) };
    game.startMatch(matchSettings);
  });
  $('#to-menu').addEventListener('click', toGarage);
  $('#opt-sound').addEventListener('change', (e) => (game.audio.muted = !e.target.checked));
  const hq = $('#opt-hq');
  hq.checked = game.hq;
  hq.addEventListener('change', () => {
    game.hq = hq.checked;
    game.applyQuality();
  });
  addEventListener('keydown', (e) => {
    if (e.code === 'Escape' || e.code === 'KeyP') {
      if (game.state === 'playing') game.pause(true);
      else if (game.state === 'paused') game.pause(false);
    }
    if (e.code === 'Enter' && game.state === 'menu' && !$('#menu').classList.contains('hidden')) start();
  });
  // Pause automatically when the tab/app goes to the background (phones)
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && game.state === 'playing') game.pause(true);
  });

  game.showGarage(settings);
  game.start();
  $('#loading').classList.add('hidden');
  $('#menu').classList.remove('hidden');

  // Handy for debugging from the browser console
  window.robotSim = game;
}

boot();

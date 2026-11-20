// Drive: two on-screen sticks for a phone. The panel publishes them as the gamepad's /joy
// (sandro_gui/touch.py), so touching a stick is holding L1: the robot drives only while a
// finger is down and this page keeps posting; it stops by itself when the posts stop.
// A routine button is the same with the panel moving the sticks (routines.py): the page
// only says which routine the finger holds.

import { $, h, replace, badge, signed } from "./util.js";

const PERIOD_MS = 80;          // a post every 80 ms; the panel stops the robot 350 ms after the last
const DEADZONE = 0.1;
const MIN_PRESS_MS = 250;      // a tap lasts at least this long, so a few /joy messages carry it
const client = Math.random().toString(36).slice(2) + Date.now().toString(36);

let ctx, timer = null, visible = false;
let inFlight = false, releaseOwed = false, lastError = null, rtt = null, lastOk = 0;
const sticks = { left: { x: 0, y: 0, pointer: null }, right: { x: 0, y: 0, pointer: null } };
const pressed = new Map();     // button name -> time it may be let go
let routine = null, plays = 0; // the routine button held: { name, play }

function shaped(value) {
  const size = Math.abs(value);
  if (size < DEADZONE) return 0;
  return Math.sign(value) * Math.min(1, (size - DEADZONE) / (1 - DEADZONE));
}

function holding() {
  return routine !== null || sticks.left.pointer !== null || sticks.right.pointer !== null || pressed.size > 0;
}

function payload() {
  const hold = holding();
  const speed = Number($("#drive-speed").value) / 100;
  if (routine) return { client, hold, routine: routine.name, play: routine.play, speed };
  return { client, hold,
    lx: hold ? shaped(sticks.left.x) : 0, ly: hold ? shaped(sticks.left.y) : 0,
    rx: hold ? shaped(sticks.right.x) : 0, ry: hold ? shaped(sticks.right.y) : 0,
    buttons: [...pressed.keys()], speed };
}

async function post() {
  const now = performance.now();
  for (const [name, until] of pressed) if (until !== Infinity && now >= until) pressed.delete(name);
  const body = payload();
  if (!body.hold && !releaseOwed) return;
  if (inFlight) return;        // never queue: a late command is worse than a missing one
  inFlight = true;
  const abort = new AbortController();
  const giveUp = setTimeout(() => abort.abort(), 300);
  try {
    const response = await fetch("/api/touch", { method: "POST", signal: abort.signal,
      headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || `${response.status}`);
    rtt = performance.now() - now;
    lastOk = performance.now();
    lastError = null;
    if (!body.hold) releaseOwed = false;
  } catch (error) {
    lastError = error.name === "AbortError" ? "no answer from the robot within 0.3 s" : error.message;
  } finally {
    clearTimeout(giveUp);
    inFlight = false;
    render();
  }
}

function release() {
  for (const stick of Object.values(sticks)) Object.assign(stick, { x: 0, y: 0, pointer: null });
  pressed.clear();
  routine = null;
  releaseOwed = true;
  for (const zone of document.querySelectorAll(".stick-zone")) moveKnob(zone, 0, 0, false);
  for (const button of document.querySelectorAll(".drive-buttons button, .drive-routines button")) button.classList.remove("held");
  // The page may be going away: this one is sent even if the tab is frozen right after.
  navigator.sendBeacon?.("/api/touch", new Blob([JSON.stringify({ client, hold: false })], { type: "application/json" }));
}

function moveKnob(zone, x, y, active) {
  const knob = zone.querySelector(".stick-knob");
  const reach = (zone.clientWidth - knob.offsetWidth) / 2;
  knob.style.transform = `translate(${x * reach}px, ${-y * reach}px)`;
  zone.classList.toggle("active", active);
}

function stickZone(id, side) {
  const zone = $(id);
  const stick = sticks[side];
  const update = (event) => {
    const box = zone.getBoundingClientRect();
    const reach = box.width / 2 - zone.querySelector(".stick-knob").offsetWidth / 2;
    let x = (event.clientX - (box.left + box.width / 2)) / reach;
    let y = -(event.clientY - (box.top + box.height / 2)) / reach;
    const size = Math.hypot(x, y);
    if (size > 1) { x /= size; y /= size; }
    stick.x = x;
    stick.y = y;
    moveKnob(zone, x, y, true);
  };
  zone.addEventListener("pointerdown", (event) => {
    if (stick.pointer !== null) return;
    event.preventDefault();
    zone.setPointerCapture(event.pointerId);
    stick.pointer = event.pointerId;
    update(event);
    post();
  });
  zone.addEventListener("pointermove", (event) => {
    if (event.pointerId === stick.pointer) update(event);
  });
  const up = (event) => {
    if (event.pointerId !== stick.pointer) return;
    Object.assign(stick, { x: 0, y: 0, pointer: null });
    moveKnob(zone, 0, 0, false);
    releaseOwed = true;
    post();
  };
  zone.addEventListener("pointerup", up);
  zone.addEventListener("pointercancel", up);
  zone.addEventListener("lostpointercapture", up);
  zone.addEventListener("contextmenu", (event) => event.preventDefault());
}

function holdButton(name, label, title) {
  const button = h("button", { title }, label);
  let down = 0;
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    down = performance.now();
    pressed.set(name, Infinity);
    button.classList.add("held");
    post();
  });
  const up = () => {
    if (!pressed.has(name)) return;
    pressed.set(name, down + MIN_PRESS_MS);
    button.classList.remove("held");
    releaseOwed = true;
  };
  button.addEventListener("pointerup", up);
  button.addEventListener("pointercancel", up);
  button.addEventListener("lostpointercapture", up);
  button.addEventListener("contextmenu", (event) => event.preventDefault());
  return button;
}

// A routine plays while its button is held, from the beginning on every press.
function routineButton(spec) {
  const button = h("button", { title: spec.note, "data-routine": spec.name },
    h("span", { class: "routine-bar" }), h("span", { class: "routine-name" }, spec.label),
    h("span", { class: "routine-time mono" }, `${spec.duration_s.toFixed(0)} s`));
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (routine) return;
    button.setPointerCapture(event.pointerId);
    routine = { name: spec.name, play: ++plays };
    button.classList.add("held");
    replace($("#drive-routine-note"), h("b", {}, spec.label), ` ${spec.note}`);
    post();
  });
  const up = () => {
    if (routine?.name !== spec.name) return;
    routine = null;
    button.classList.remove("held");
    releaseOwed = true;
    post();
  };
  button.addEventListener("pointerup", up);
  button.addEventListener("pointercancel", up);
  button.addEventListener("lostpointercapture", up);
  button.addEventListener("contextmenu", (event) => event.preventDefault());
  return button;
}

function renderRoutines(playing) {
  for (const button of document.querySelectorAll(".drive-routines button")) {
    const mine = playing && playing.name === button.dataset.routine;
    const spec = ctx.state.info.touch.routines.find((r) => r.name === button.dataset.routine);
    button.querySelector(".routine-bar").style.width = mine ? `${100 * playing.t / playing.duration_s}%` : "0";
    button.querySelector(".routine-time").textContent = mine
      ? (playing.done ? "done" : `${playing.t.toFixed(1)} / ${playing.duration_s.toFixed(0)} s`)
      : `${spec.duration_s.toFixed(0)} s`;
  }
}

function render() {
  if (!visible) return;
  const fast = ctx.state.fast || {}, slow = ctx.state.slow || {};
  const touch = fast.touch;
  const active = slow.ros?.teleop?.active;
  const cmd = fast.ros?.cmd_vel;
  const mine = touch?.driving && touch.client === client;
  renderRoutines(mine ? touch.routine : null);
  const stale = holding() && performance.now() - lastOk > 500;
  replace($("#drive-status"),
    !touch ? badge("not available: the panel has no ROS", "error")
      : lastError && holding() ? badge(lastError, "error")
        : stale ? badge("link lost: the robot has stopped", "error")
          : mine && touch.routine ? badge(touch.routine.done ? "routine done: let go" : `playing ${touch.routine.name}`, "ok")
          : mine ? badge("driving", "ok")
            : touch.driving ? badge("another browser is driving", "warn")
              : badge("touch a stick to drive", "info"),
    h("span", {}, active ? `controller ${active}` : "no controller running"),
    rtt !== null && holding() ? h("span", { class: "muted mono" }, `${rtt.toFixed(0)} ms`) : null);
  replace($("#drive-command"), cmd && mine
    ? h("span", { class: "mono" }, `forward ${signed(cmd.forward, 2)} m/s · yaw ${signed(cmd.yaw, 2)} rad/s · height ${signed(cmd.height_rate, 3)} m/s · roll ${signed(cmd.roll, 2)} rad`)
    : h("span", { class: "muted" }, "No command: nothing is touched, or teleop is not running (start the stack with teleop)."));
  $("#drive-speed-value").textContent = `${$("#drive-speed").value} %`;
}

export function init(context) {
  ctx = context;
  stickZone("#stick-left", "left");
  stickZone("#stick-right", "right");
  $("#drive-buttons").append(
    holdButton("cross", "Hop", "One hop (policies with a hop input). The same as L1 + Cross."),
    holdButton("triangle", "Next policy (hold)", "Keep holding until the switch is done: letting go aborts it. The same as L1 + Triangle."),
    holdButton("r3", "Height reset (hold)", "Back to the running policy's initial height. The same as L1 + R3."));
  const routines = ctx.state.info?.touch?.routines || [];
  $("#drive-routines").append(...routines.map(routineButton));
  $("#drive-routines-block").hidden = routines.length === 0;
  const speed = $("#drive-speed");
  const top = Math.round(100 * (ctx.state.info?.touch?.max_speed ?? 1));
  speed.max = top;
  speed.value = Math.min(top, Number(localStorage.getItem("drive-speed") || 50));
  speed.addEventListener("input", () => { localStorage.setItem("drive-speed", speed.value); render(); });
  ctx.on("fast", render);
  document.addEventListener("visibilitychange", () => { if (document.visibilityState !== "visible") release(); });
  window.addEventListener("pagehide", release);
  window.addEventListener("blur", () => { if (holding()) release(); });
}

export function show() {
  visible = true;
  clearInterval(timer);
  timer = setInterval(post, PERIOD_MS);
  render();
}

export function hide() {
  if (!visible) return;
  visible = false;
  if (holding()) release();
  clearInterval(timer);
  timer = null;
  post();
}

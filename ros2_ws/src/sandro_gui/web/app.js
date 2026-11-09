// Shell: live stream, tabs, header status and the E-stop. Pages subscribe to events.

import { $, h, replace, dot } from "./util.js";
import * as overview from "./overview.js";
import * as run from "./run.js";
import * as tune from "./tune.js";
import * as imu from "./imu.js";
import * as pad from "./pad.js";
import * as guide from "./guide.js";

const PAGES = { overview, run, tune, imu, pad, guide };
const listeners = {};
const state = { info: null, fast: null, slow: null, logs: [], logSeq: 0, page: null };

function on(event, fn) {
  (listeners[event] ||= []).push(fn);
}

function emit(event, data) {
  for (const fn of listeners[event] || []) {
    try {
      fn(data);
    } catch (error) {
      console.error(event, error);
    }
  }
}

async function api(path, body) {
  const response = await fetch(`/api/${path}`, body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  let data = {};
  try {
    data = await response.json();
  } catch {
    data = {};
  }
  if (!response.ok) {
    const error = new Error(data.error || `${response.status} ${response.statusText}`);
    error.status = response.status;
    error.data = data;
    throw error;
  }
  return data;
}

function toast(text, level = "info") {
  const box = $("#toasts");
  const item = h("div", { class: `toast ${level}` }, text);
  box.append(item);
  setTimeout(() => item.classList.add("gone"), level === "error" ? 7000 : 3500);
  setTimeout(() => item.remove(), level === "error" ? 7600 : 4100);
}

// A modal yes/no. Resolves true when confirmed.
function confirm({ title, body, ok = "Confirm", danger = false, check = null }) {
  const dialog = $("#dialog");
  return new Promise((resolve) => {
    const tick = check ? h("input", { type: "checkbox", id: "dialog-check" }) : null;
    const yes = h("button", { class: danger ? "danger" : "primary", disabled: !!check }, ok);
    const no = h("button", {}, "Cancel");
    if (tick) tick.addEventListener("change", () => { yes.disabled = !tick.checked; });
    replace(dialog,
      h("h3", {}, title),
      h("div", { class: "dialog-body" }, body),
      tick ? h("label", { class: "check" }, tick, check) : null,
      h("div", { class: "dialog-actions" }, no, yes));
    const close = (value) => { dialog.close(); resolve(value); };
    yes.addEventListener("click", () => close(true));
    no.addEventListener("click", () => close(false));
    dialog.addEventListener("cancel", () => resolve(false), { once: true });
    dialog.showModal();
  });
}

async function act(path, body, success) {
  try {
    const result = await api(path, body || {});
    if (success !== false) toast(success || result.message || "done", "ok");
    return result;
  } catch (error) {
    toast(error.message, "error");
    throw error;
  }
}

const ctx = { state, on, emit, api, act, toast, confirm };

// Stream -----------------------------------------------------------------

let source = null;
let lastMessage = 0;

function connect() {
  source?.close();
  source = new EventSource(`/api/stream?after=${state.logSeq}`);
  source.addEventListener("open", () => setOnline(true));
  source.addEventListener("error", () => setOnline(false));
  source.addEventListener("fast", (e) => {
    lastMessage = performance.now();
    state.fast = JSON.parse(e.data);
    emit("fast", state.fast);
  });
  source.addEventListener("slow", (e) => {
    lastMessage = performance.now();
    state.slow = JSON.parse(e.data);
    emit("slow", state.slow);
    header(state.slow);
  });
  source.addEventListener("log", (e) => {
    const fresh = JSON.parse(e.data).filter((entry) => entry.seq > state.logSeq);
    if (!fresh.length) return;
    state.logSeq = fresh[fresh.length - 1].seq;
    state.logs.push(...fresh);
    if (state.logs.length > 5000) state.logs.splice(0, state.logs.length - 5000);
    emit("log", fresh);
  });
}

function setOnline(online) {
  document.body.classList.toggle("offline", !online);
}

setInterval(() => {
  if (lastMessage && performance.now() - lastMessage > 4000) {
    setOnline(false);
    connect();
    lastMessage = performance.now();
  }
}, 1000);

// Header -----------------------------------------------------------------

const CHIPS = [["stack", "Stack"], ["can", "CAN"], ["drives", "Drives"], ["imu", "IMU"],
  ["controller", "Ctrl"], ["pad", "Pad"]];

function header(slow) {
  const checks = Object.fromEntries((slow.checks || []).map((c) => [c.id, c]));
  replace($("#chips"), CHIPS.map(([id, label]) => {
    const check = checks[id];
    if (!check) return null;
    return h("button", { class: `chip ${check.level}`, title: `${check.label}: ${check.value}${check.detail ? "\n" + check.detail : ""}`, onclick: () => go("overview") },
      dot(check.level), h("span", { class: "chip-label" }, label),
      check.level === "off" ? null : h("span", { class: "chip-value" }, check.short || check.value));
  }));
  $("#host").textContent = slow.system?.host || "";
  const estop = $("#estop");
  estop.disabled = !state.info?.ros;
}

// Pages ------------------------------------------------------------------

function go(page) {
  if (!PAGES[page]) page = "overview";
  if (location.hash !== `#${page}`) history.replaceState(null, "", `#${page}`);
  state.page = page;
  for (const button of document.querySelectorAll("#tabs button")) {
    button.classList.toggle("active", button.dataset.page === page);
  }
  for (const [name, module] of Object.entries(PAGES)) {
    const section = $(`#page-${name}`);
    const visible = name === page;
    section.hidden = !visible;
    if (visible) module.show?.();
    else module.hide?.();
  }
}

async function start() {
  for (const button of document.querySelectorAll("#tabs button")) {
    button.addEventListener("click", () => go(button.dataset.page));
  }
  window.addEventListener("hashchange", () => go(location.hash.slice(1)));
  $("#estop").addEventListener("click", async () => {
    try {
      const result = await api("estop", {});
      toast(`E-STOP: ${result.message}`, "error");
    } catch (error) {
      toast(`E-STOP failed: ${error.message}. Use the power cut-off.`, "error");
    }
  });
  for (;;) {
    try {
      state.info = await api("info");
      break;
    } catch {
      setOnline(false);
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
  document.body.classList.toggle("simulation", state.info.simulation);
  try {
    const logs = await api("logs?after=0&limit=2000");
    state.logs = logs;
    state.logSeq = logs.length ? logs[logs.length - 1].seq : 0;
  } catch {
    // the stream catches up
  }
  for (const module of Object.values(PAGES)) module.init(ctx);
  emit("log", state.logs);
  go(location.hash.slice(1) || "overview");
  connect();
}

start();

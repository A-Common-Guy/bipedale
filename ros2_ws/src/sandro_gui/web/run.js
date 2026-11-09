// Run: start the stack, arm the drives, start controllers, recover after an E-stop or a fall, read the log.

import { $, h, replace, dot, badge, num, duration, clock, bytes } from "./util.js";

let ctx;
const STORE = "sandro.run.form";
const form = Object.assign({
  hardware: null, mode: "stack", controller: "whole_body_controller", teleop: true, overrides: true,
  state_source: "encoder", encoder_period_ms: "", extra: "",
}, JSON.parse(localStorage.getItem(STORE) || "{}"));
if (!["stack", "imu"].includes(form.mode)) form.mode = "stack";
// 5 was the stored default before the launch chose the period from the state source.
if (form.encoder_period_ms === 5) form.encoder_period_ms = "";
const filter = { level: "info", text: "", follow: true };
const LEVELS = { debug: 0, info: 1, warn: 2, error: 3, fatal: 4 };
let previewTimer = null;
const keys = { form: null, launch: null, flow: null, tables: null };
// Requests this tab has in flight, by action key: their buttons are disabled and spin.
const inflight = new Set();
let chosen = null;

function save() {
  localStorage.setItem(STORE, JSON.stringify(form));
}

function request() {
  return { ...form, hardware: form.hardware || ctx.state.info.hardware };
}

function simulation() {
  return request().hardware === "mujoco";
}

// Actions ----------------------------------------------------------------

function rerender() {
  if (ctx.state.slow) render(ctx.state.slow, true);
}

async function track(key, fn) {
  if (inflight.has(key)) return undefined;
  inflight.add(key);
  try {
    rerender();
    return await fn();
  } catch {
    return undefined;
  } finally {
    inflight.delete(key);
    rerender();
  }
}

function button(key, label, onclick, cls = "small", disabled = false) {
  const busy = inflight.has(key);
  const serverBusy = !!ctx.state.slow?.busy && key !== "launch";
  return h("button", { class: `${cls}${busy ? " busy" : ""}`, disabled: disabled || busy || (serverBusy && !busy), onclick: () => track(key, onclick) }, label);
}

const info = (name) => ctx.state.info.controller_info?.[name] || {};

function startText(name) {
  const c = info(name);
  if (c.start_lying) {
    return `Lay the robot down on its side or back, clear of anything it can hit. ${name} stays limp until the robot rests, then stands it up; there is no fall cut-off until it is upright.`;
  }
  if (c.type === "RlPolicyController") {
    return `Hold the robot upright: the legs move to the stand pose at once and ${name} engages once the body is upright.`;
  }
  return `Hold the robot upright: the legs stand at once and ${name} balances once the robot is upright and still.`;
}

const SECURED = "The robot is secured and the power cut-off is in reach";

function session() {
  return ctx.state.slow?.session || {};
}

async function arm(component) {
  const first = !session().armed_once;
  if (first) {
    const ok = await ctx.confirm({
      title: `Arm ${component}?`,
      body: h("p", {}, "The drives enter closed loop at zero torque with no controller, and an E-stop latch clears. This is asked once per launch."),
      ok: "Arm", danger: true, check: SECURED,
    });
    if (!ok) return;
  }
  await ctx.act("hardware/activate", { name: component, secured: first });
}

async function armAndStart(name, title) {
  const ok = await ctx.confirm({
    title: `${title} ${name}?`,
    body: h("div", {}, h("p", {}, "The drives are armed, then ", h("span", { class: "mono" }, name), " starts."), h("p", {}, startText(name))),
    ok: title, danger: true, check: session().armed_once ? null : SECURED,
  });
  if (ok) await ctx.act("arm_and_start", { controller: name, secured: !session().armed_once }, `${name} active`);
}

function owners(except) {
  return (ctx.state.slow?.ros?.controllers || []).filter((c) => c.owns_commands && c.name !== except).map((c) => c.name);
}

async function swap(name, running) {
  const rl = info(name).type === "RlPolicyController";
  const ok = await ctx.confirm({
    title: `Swap to ${name}?`,
    body: h("div", {},
      h("p", {}, `${running.join(", ")} owns the drives; ${name} takes over in one controller switch.`),
      h("p", {}, rl ? `It is told the switch is a handover (arm_handover): if the body is upright it engages at once, from whatever pose the robot is in; otherwise it starts its stand phase.`
        : "It starts from whatever pose the robot is in."),
      h("p", {}, "Between the RL policies the gamepad's L1 + Triangle is the soft way: it settles the robot first.")),
    ok: "Swap now", danger: true,
  });
  if (ok) await ctx.act("controller/activate", { name, swap: true }, `${name} active`);
}

async function startController(name) {
  const running = owners(name);
  if (running.length && !simulation()) return swap(name, running);
  if (!simulation()) {
    const ok = await ctx.confirm({ title: `Start ${name}?`, body: h("p", {}, startText(name)), ok: "Start", danger: true });
    if (!ok) return;
  }
  try {
    await ctx.api("controller/activate", { name, swap: simulation() && running.length > 0 });
    ctx.toast(`${name} active`, "ok");
  } catch (error) {
    if (error.status === 409 && error.data.conflict) return swap(name, error.data.conflict);
    ctx.toast(error.message, "error");
  }
}

async function restart(name) {
  const ok = await ctx.confirm({
    title: `Restart ${name}?`,
    body: h("div", {}, h("p", {}, "It is deactivated and activated again, so it starts over from its first phase."), h("p", {}, startText(name))),
    ok: "Restart", danger: true,
  });
  if (ok) await ctx.act("controller/restart", { name }, `${name} restarted`);
}

async function clearErrors() {
  const ok = await ctx.confirm({
    title: "Clear drive errors?",
    body: h("p", {}, "Clear_Errors goes to every drive, without arming. It clears every error bit, so read the Drives table on the Overview first: a fault whose cause is still there comes back."),
    ok: "Clear errors", danger: true,
  });
  if (ok) await ctx.act("clear_errors", {});
}

// Start form -------------------------------------------------------------

function segmented(options, value, onchange) {
  return h("div", { class: "segmented" }, options.map(([key, label, title]) =>
    h("button", { class: key === value ? "active" : "", title, onclick: () => onchange(key) }, label)));
}

function renderForm() {
  const info = ctx.state.info;
  const sim = simulation();
  const imu = !sim && form.mode === "imu";
  const select = h("select", { disabled: imu, onchange: (e) => { form.controller = e.target.value; changed(); } },
    ["none", ...info.controllers].map((name) => h("option", { value: name, selected: name === form.controller },
      name === "none" ? "none" : name)));
  const check = (key, label, title, disabled = false) => h("label", { class: "check", title },
    h("input", { type: "checkbox", checked: form[key], disabled, onchange: (e) => { form[key] = e.target.checked; changed(); } }), label);
  replace($("#start-form"),
    info.available_hardware.length > 1 ? h("div", { class: "field" }, h("span", { class: "field-label" }, "Target"),
      segmented([["real", "Robot"], ["mujoco", "Simulation"]], request().hardware, (v) => { form.hardware = v; changed(true); })) : null,
    h("p", { class: "hint" }, sim ? "MuJoCo with the latency emulation; nothing physical moves. The controller starts with the simulation."
      : imu ? "Diagnostic: the BNO085 plugin alone, no drive plugin is loaded. Use it to check the IMU with the drives switched off."
        : "One launch for the session. The drives come up configured but not armed; joint states and the IMU are live at once, and the controller is loaded ready to start. Arm and start it from here once the stack is ready."),
    h("div", { class: "field" }, h("span", { class: "field-label" }, sim ? "Controller" : "Preload"), select),
    h("div", { class: "field checks-row" },
      check("teleop", "Gamepad teleop", "joy_node, joy_teleop and joy_estop", imu),
      sim ? null : check("overrides", "Robot overrides", info.overrides || "")),
    h("details", { class: "advanced", open: imu }, h("summary", {}, "Advanced"),
      sim ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "Launch"),
        segmented([["stack", "Robot stack", "The whole stack, drives not armed"],
          ["imu", "IMU check", "Only the BNO085 plugin: no drive plugin is loaded"]],
        form.mode, (v) => { form.mode = v; changed(true); })),
      imu ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "State source"),
        segmented([["encoder", "encoder broadcast", "Joint state from the drives' 5 ms broadcast"],
          ["mit", "MIT reply", "Joint state from each drive's reply to every 300 Hz command, about 2.3 ms old; the broadcast slows to 20 ms"]],
        form.state_source, (v) => { form.state_source = v; changed(true); })),
      sim || imu ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "Encoder period"),
        h("input", { type: "number", min: 1, max: 100, value: form.encoder_period_ms, class: "narrow",
          placeholder: form.state_source === "mit" ? "auto, 20" : "auto, 5",
          onchange: (e) => { form.encoder_period_ms = e.target.value === "" ? "" : Number(e.target.value); changed(); } }),
        h("span", { class: "unit" }, "ms")),
      h("div", { class: "field" }, h("span", { class: "field-label" }, "Extra arguments"),
        h("input", { type: "text", value: form.extra, placeholder: "name:=value …", spellcheck: false,
          oninput: (e) => { form.extra = e.target.value; changed(); } }))),
    h("pre", { id: "command-preview", class: "command" }, "…"));
  $("#launch-start").textContent = sim ? "Start simulation" : imu ? "Start IMU check" : "Start stack";
  preview();
}

function changed(rerenderForm = false) {
  save();
  if (rerenderForm) renderForm();
  else preview();
}

function preview() {
  clearTimeout(previewTimer);
  previewTimer = setTimeout(async () => {
    const out = $("#command-preview");
    if (!out) return;
    try {
      out.textContent = (await ctx.api("launch/preview", request())).command;
      out.classList.remove("error");
    } catch (error) {
      out.textContent = error.message;
      out.classList.add("error");
    }
  }, 150);
}

// Stack ------------------------------------------------------------------

const RUNNING = ["running", "starting", "stopping", "external"];

function renderLaunch(slow, stack, force) {
  const launch = slow.launch;
  const run = launch.run;
  const running = RUNNING.includes(launch.state);
  const uptime = $("#launch-uptime");
  if (uptime) uptime.textContent = run ? ` · up ${duration(run.uptime_s)} · pid ${run.pid}` : "";
  const key = JSON.stringify([launch.state, run?.pid, launch.external?.length, launch.last_exit?.at, stack?.phase, stack?.value, slow.waiting, run?.died, [...inflight]]);
  if (!force && key === keys.launch) return;
  keys.launch = key;
  $("#start-form").hidden = running;
  const startBusy = inflight.has("launch");
  $("#launch-start").disabled = running || startBusy;
  $("#launch-start").classList.toggle("busy", startBusy && !running);
  $("#launch-stop").disabled = !running || launch.state === "stopping";
  $("#launch-stop").textContent = launch.state === "stopping" ? "Stopping…" : "Stop";
  const waiting = launch.state === "starting" && slow.waiting?.length;
  replace($("#launch-status"),
    h("div", { class: "status-line" }, dot(running ? stack?.level : "off"), h("strong", {}, running ? stack?.value || launch.state : launch.state),
      h("span", { id: "launch-uptime", class: "muted" }, run ? ` · up ${duration(run.uptime_s)} · pid ${run.pid}` : "")),
    waiting ? h("p", { class: "hint" }, `Waiting for ${slow.waiting.join(", ")}.`) : null,
    running && stack?.phase === "died" ? h("p", { class: "warn-text" }, stack.detail) : null,
    run?.died?.length && stack?.phase !== "died" ? h("p", { class: "warn-text" }, `Died in this launch: ${[...new Set(run.died)].join(", ")} (see the log).`) : null,
    run ? h("pre", { class: "command" }, run.command) : null,
    run ? h("a", { href: `/api/runs/${run.log}`, class: "small-link" }, run.log) : null,
    launch.state === "external" ? h("p", { class: "hint" },
      "Started outside the panel (deploy/pi/pi launch?). Its output is not captured here; Stop sends it SIGINT.",
      h("pre", { class: "command" }, launch.external[0]?.command || "")) : null,
    !running && launch.last_exit ? h("p", { class: launch.last_exit.code ? "warn-text" : "muted" },
      `Last run exited ${launch.last_exit.code === null ? "" : `with code ${launch.last_exit.code} `}at ${clock(launch.last_exit.at)} · `,
      h("a", { href: `/api/runs/${launch.last_exit.log}`, class: "small-link" }, launch.last_exit.log)) : null);
  const formKey = JSON.stringify([running, launch.last_exit?.at]);
  if (formKey !== keys.form && !running) renderForm();
  keys.form = formKey;
}

// The one-button path: arm, start, recover.
function renderFlow(slow, stack, force) {
  const ros = slow.ros || {};
  const component = ctx.state.info.component;
  const system = (ros.hardware || []).find((hw) => hw.name === component);
  const controllers = ros.controllers || [];
  const running = controllers.filter((c) => c.owns_commands);
  const s = session();
  const names = ctx.state.info.controllers;
  const fallback = s.last_controller || slow.launch.run?.request?.controller;
  const target = chosen && names.includes(chosen) ? chosen : names.includes(fallback) ? fallback : names[0];
  const key = JSON.stringify([stack?.phase, ros.manager, system?.state, running.map((c) => [c.name, c.fault?.state]), s, target, [...inflight], slow.busy]);
  if (!force && key === keys.flow) return;
  keys.flow = key;
  const box = $("#flow");
  const live = RUNNING.includes(slow.launch.state) && slow.launch.state !== "stopping";
  if (!live || !ros.manager || stack?.phase === "imu" || (!system && !simulation())) {
    replace(box);
    box.hidden = true;
    return;
  }
  box.hidden = false;
  const picker = h("select", { class: "flow-select", onchange: (e) => { chosen = e.target.value; rerender(); } },
    names.map((n) => h("option", { value: n, selected: n === target }, n)));
  const armed = system?.state === "active" || simulation();
  const rows = [];
  if (slow.busy) rows.push(h("p", { class: "hint" }, badge(`${slow.busy}…`, "info")));
  const fallen = running.find((c) => c.fault);
  if (fallen) {
    rows.push(h("div", { class: "status-line" }, dot("error"), h("strong", { class: "warn-text" }, `${fallen.name} ${fallen.fault.state}`), h("span", { class: "muted" }, ` · ${fallen.fault.text}`)),
      h("div", { class: "button-row" },
        button(`restart:${fallen.name}`, `Restart ${fallen.name}`, () => restart(fallen.name), "danger"),
        button(`stop:${fallen.name}`, "Stop it", () => ctx.act("controller/deactivate", { name: fallen.name })),
        simulation() ? null : button("disarm", "Disarm", () => ctx.act("hardware/deactivate", { name: component }))));
  } else if (running.length) {
    const name = running[0].name;
    rows.push(h("div", { class: "status-line" }, dot("ok"), h("strong", {}, `${name} running`)),
      h("div", { class: "button-row" },
        button(`stop:${name}`, `Stop ${name}`, () => ctx.act("controller/deactivate", { name })),
        simulation() ? null : button("disarm", "Disarm", () => ctx.act("hardware/deactivate", { name: component })),
        h("span", { class: "flow-or" }, "or swap to"), picker,
        target !== name ? button(`start:${target}`, "Swap", () => startController(target), "danger") : null));
  } else if (armed) {
    rows.push(h("div", { class: "status-line" }, dot("warn"), h("strong", {}, "Drives armed, no controller")),
      h("div", { class: "button-row" }, picker,
        button(`start:${target}`, `Start ${target}`, () => startController(target), "danger"),
        simulation() ? null : button("disarm", "Disarm", () => ctx.act("hardware/deactivate", { name: component }))));
  } else if (stack?.phase === "estopped") {
    rows.push(h("div", { class: "status-line" }, dot("warn"), h("strong", {}, "E-stopped"), h("span", { class: "muted" }, ` · drives ${system?.state}`)),
      h("p", { class: "hint" }, "The drives latched the E-stop; arming clears the latch. Re-arm and restart does both in one step."),
      h("div", { class: "button-row" }, picker,
        button(`arm_start:${target}`, `Re-arm and restart ${target}`, () => armAndStart(target, "Re-arm and restart"), "danger"),
        button("arm", "Re-arm only", () => arm(component))));
  } else {
    const off = system?.state !== "inactive";
    rows.push(h("div", { class: "status-line" }, dot(off ? "warn" : "off"), h("strong", {}, `Drives ${system?.state || "not listed"}`),
      h("span", { class: "muted" }, off ? " · a drive did not configure: see the Drives table" : " · not armed")),
      h("div", { class: "button-row" },
        button("arm", "Arm", () => arm(component), "primary"),
        h("span", { class: "flow-or" }, "or"), picker,
        button(`arm_start:${target}`, `Arm and start ${target}`, () => armAndStart(target, "Arm and start"), "danger")));
  }
  replace(box, h("h3", {}, "Drive the robot"), rows);
}

async function start() {
  await track("launch", () => ctx.act("launch/start", request(), "launch started"));
}

async function stop() {
  const running = owners();
  if (running.length && !simulation()) {
    const ok = await ctx.confirm({
      title: "Stop the stack?",
      body: h("p", {}, `${running.join(", ")} is driving the robot: stopping the stack idles the drives and a balancing robot falls.`),
      ok: "Stop", danger: true,
    });
    if (!ok) return;
  }
  await ctx.act("launch/stop", {}, "stopping").catch(() => {});
}

// Hardware and controllers -----------------------------------------------

const short = (type) => (type || "").split("/").pop();

function controllerRow(name, c, manager, armed) {
  const state = c?.fault ? c.fault.state : c?.state || "not loaded";
  const level = c?.fault ? "error" : { active: "ok", inactive: "", unconfigured: "warn" }[state] ?? "off";
  const broadcaster = (c?.type || "").includes("broadcaster");
  const go = (key, label, fn, cls = "small") => button(key, label, fn, cls, !manager);
  let actions = null;
  if (!broadcaster) {
    if (c?.state === "active") {
      actions = [c.fault ? go(`restart:${name}`, "Restart", () => restart(name), "small danger") : null,
        go(`stop:${name}`, "Stop", () => ctx.act("controller/deactivate", { name }))];
    } else {
      actions = [armed ? go(`start:${name}`, "Start", () => startController(name))
        : go(`arm_start:${name}`, "Arm and start", () => armAndStart(name, "Arm and start")),
      go(`configure:${name}`, state === "inactive" ? "Re-configure" : "Load", () => ctx.act("controller/configure", { name }))];
    }
  }
  return h("tr", { class: broadcaster ? "dim-row" : "" },
    h("td", {}, h("span", { class: "mono" }, name)),
    h("td", { class: "muted" }, short(c?.type)),
    h("td", {}, badge(state, level)),
    h("td", { class: "right actions" }, actions));
}

function renderTables(slow, force) {
  const ros = slow.ros || {};
  const manager = !!ros.manager;
  const component = ctx.state.info.component;
  const hardware = ros.hardware || [];
  const armed = simulation() || hardware.some((hw) => hw.name === component && hw.state === "active");
  const key = JSON.stringify([manager, (ros.controllers || []).map((c) => [c.name, c.state, c.fault?.state]),
    hardware.map((hw) => [hw.name, hw.state]), [...inflight], slow.busy]);
  if (force || key !== keys.tables) {
    keys.tables = key;
    const listed = Object.fromEntries((ros.controllers || []).map((c) => [c.name, c]));
    const configured = ctx.state.info.controllers;
    const names = [...configured, ...Object.keys(listed).filter((n) => !configured.includes(n))];
    names.sort((a, b) => (listed[a]?.type || "").includes("broadcaster") - (listed[b]?.type || "").includes("broadcaster"));
    replace($("#controllers"),
      manager ? null : h("p", { class: "hint" }, "No controller manager: start the stack (or the simulation)."),
      h("table", { class: "table" }, h("tbody", {}, names.map((n) => controllerRow(n, listed[n], manager, armed)))));
    replace($("#hardware"), hardware.length ? h("table", { class: "table" }, h("tbody", {}, hardware.map((hw) => {
      const active = hw.state === "active";
      return h("tr", {},
        h("td", {}, h("span", { class: "mono" }, hw.name)),
        h("td", { class: "muted" }, hw.plugin?.split("/").pop()),
        h("td", {}, badge(hw.state, active ? "ok" : "warn")),
        h("td", { class: "right actions" }, hw.type === "system"
          ? (active ? button("disarm", "Disarm", () => ctx.act("hardware/deactivate", { name: hw.name }))
            : button("arm", "Arm", () => arm(hw.name), "small primary"))
          : null));
    }))) : h("p", { class: "hint" }, "Hardware components appear once the controller manager runs."));
  }

  const diagnostics = ros.diagnostics || {};
  const active = (ros.controllers || []).find((c) => c.owns_commands);
  const shown = active && diagnostics[active.name] ? active.name : Object.keys(diagnostics)[0];
  const diag = shown && diagnostics[shown];
  const fault = active && active.name === shown ? active.fault : null;
  replace($("#diagnostics"), diag ? [
    h("div", { class: "status-line" }, fault ? dot("error") : null, h("span", { class: "mono" }, shown), " · ",
      h("strong", { class: fault ? "warn-text" : "" }, fault ? `${fault.state}: ${fault.text}` : diag.message || "—")),
    h("div", { class: "kv" }, Object.entries(diag.values).map(([k, v]) =>
      h("div", { class: "kv-item" }, h("span", { class: "kv-key" }, k), h("span", { class: "kv-value mono" }, typeof v === "number" ? num(v, Math.abs(v) >= 100 ? 1 : 3) : v)))),
  ] : h("p", { class: "hint" }, "Live values appear here while a controller publishes ~/diagnostics."));
}

function render(slow, force = false) {
  const stack = (slow.checks || []).find((c) => c.id === "stack");
  renderLaunch(slow, stack, force);
  renderFlow(slow, stack, force);
  renderTables(slow, force);
}

// Log --------------------------------------------------------------------

function passes(entry) {
  if (LEVELS[entry.level] < LEVELS[filter.level]) return false;
  if (!filter.text) return true;
  const text = filter.text.toLowerCase();
  return entry.text.toLowerCase().includes(text) || entry.source.toLowerCase().includes(text);
}

function logRow(entry) {
  return h("div", { class: `log-row ${entry.level}` },
    h("span", { class: "log-time" }, clock(entry.t)),
    h("span", { class: "log-level" }, entry.level),
    h("span", { class: "log-source" }, entry.source),
    h("span", { class: "log-text" }, entry.text));
}

function renderLog() {
  const box = $("#log");
  const rows = ctx.state.logs.filter(passes).slice(-1500).map(logRow);
  replace(box, rows);
  if (filter.follow) box.scrollTop = box.scrollHeight;
}

function appendLog(entries) {
  const box = $("#log");
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  for (const entry of entries) if (passes(entry)) box.append(logRow(entry));
  while (box.childElementCount > 1500) box.firstElementChild.remove();
  if (filter.follow && atBottom) box.scrollTop = box.scrollHeight;
}

async function runs() {
  const list = await ctx.api("runs");
  replace($("#runs"), list.length ? list.map((r) => h("a", { href: `/api/runs/${r.name}`, class: "run-link" },
    h("span", { class: "mono" }, r.name.replace("run-", "").replace(".log", "")), h("span", { class: "muted" }, bytes(r.bytes)))) :
    h("p", { class: "hint" }, "No runs recorded yet."));
}

export function init(context) {
  ctx = context;
  form.hardware ||= ctx.state.info.hardware;
  $("#launch-start").addEventListener("click", start);
  $("#launch-stop").addEventListener("click", stop);
  $("#clear-errors").addEventListener("click", () => track("clear", clearErrors));
  const level = $("#log-level");
  level.value = filter.level;
  level.addEventListener("change", () => { filter.level = level.value; renderLog(); });
  $("#log-filter").addEventListener("input", (e) => { filter.text = e.target.value; renderLog(); });
  $("#log-follow").addEventListener("change", (e) => { filter.follow = e.target.checked; if (filter.follow) renderLog(); });
  $("#runs-toggle").addEventListener("toggle", (e) => { if (e.target.open) runs().catch(() => {}); });
  renderForm();
  ctx.on("slow", (slow) => render(slow));
  ctx.on("log", appendLog);
}

export function show() {
  renderLog();
}

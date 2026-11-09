// Run: start and stop the stack, arm the drives, start controllers, read the log.

import { $, h, replace, dot, badge, num, duration, clock, bytes } from "./util.js";

let ctx;
const STORE = "sandro.run.form";
const form = Object.assign({
  hardware: null, mode: "armed", controller: "none", teleop: true, overrides: true,
  state_source: "encoder", encoder_period_ms: 5, extra: "",
}, JSON.parse(localStorage.getItem(STORE) || "{}"));
const filter = { level: "info", text: "", follow: true };
const LEVELS = { debug: 0, info: 1, warn: 2, error: 3, fatal: 4 };
let previewTimer = null;
let lastLaunch = null;

function save() {
  localStorage.setItem(STORE, JSON.stringify(form));
}

function request() {
  return { ...form, hardware: form.hardware || ctx.state.info.hardware };
}

function simulation() {
  return request().hardware === "mujoco";
}

// Start form -------------------------------------------------------------

function segmented(options, value, onchange) {
  return h("div", { class: "segmented" }, options.map(([key, label, title]) =>
    h("button", { class: key === value ? "active" : "", title, onclick: () => onchange(key) }, label)));
}

function renderForm() {
  const info = ctx.state.info;
  const sim = simulation();
  const armed = sim || form.mode === "armed";
  const controllers = ["none", ...info.controllers];
  const select = h("select", { disabled: !armed, onchange: (e) => { form.controller = e.target.value; changed(); } },
    controllers.map((name) => h("option", { value: name, selected: name === form.controller },
      name === "none" ? "none: drives at zero torque" : name)));
  const check = (key, label, title, disabled = false) => h("label", { class: "check", title },
    h("input", { type: "checkbox", checked: form[key], disabled, onchange: (e) => { form[key] = e.target.checked; changed(); } }), label);
  replace($("#start-form"),
    info.available_hardware.length > 1 ? h("div", { class: "field" }, h("span", { class: "field-label" }, "Target"),
      segmented([["real", "Robot"], ["mujoco", "Simulation"]], request().hardware, (v) => { form.hardware = v; changed(true); })) : null,
    sim ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "Mode"),
      segmented([
        ["imu", "IMU check", "Only the BNO085 plugin: no drive plugin is loaded"],
        ["drives_off", "Drives off", "Launch without the controller manager: nothing is armed"],
        ["armed", "Armed", "Controller manager up: the drives enter closed loop"]],
      form.mode, (v) => { form.mode = v; changed(true); })),
    h("p", { class: "hint" }, sim ? "MuJoCo with the latency emulation; nothing physical moves." : {
      imu: "BNO085 alone. Use it to check the IMU with the drives switched off.",
      drives_off: "The safe state of a real launch: only the E-stop node and the robot description. The CAN monitor still shows every drive.",
      armed: "The drives enter closed loop. With no controller they hold zero torque; a balancer engages once the robot is upright and still.",
    }[form.mode]),
    h("div", { class: "field" }, h("span", { class: "field-label" }, "Controller"), select),
    h("div", { class: "field checks-row" },
      check("teleop", "Gamepad teleop", "joy_node, joy_teleop and joy_estop", !armed),
      sim ? null : check("overrides", "Robot overrides", info.overrides || "")),
    h("details", { class: "advanced" }, h("summary", {}, "Advanced"),
      sim ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "State source"),
        segmented([["encoder", "encoder broadcast"], ["mit", "MIT reply"]], form.state_source, (v) => { form.state_source = v; changed(true); })),
      sim ? null : h("div", { class: "field" }, h("span", { class: "field-label" }, "Encoder period"),
        h("input", { type: "number", min: 1, max: 100, value: form.encoder_period_ms, class: "narrow",
          onchange: (e) => { form.encoder_period_ms = Number(e.target.value); changed(); } }), h("span", { class: "unit" }, "ms")),
      h("div", { class: "field" }, h("span", { class: "field-label" }, "Extra arguments"),
        h("input", { type: "text", value: form.extra, placeholder: "name:=value …", spellcheck: false,
          oninput: (e) => { form.extra = e.target.value; changed(); } }))),
    h("pre", { id: "command-preview", class: "command" }, "…"));
  preview();
}

function changed(rerender = false) {
  save();
  if (rerender) renderForm();
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

async function start() {
  const req = request();
  if (req.hardware === "real" && req.mode === "armed") {
    const ok = await ctx.confirm({
      title: "Arm the drives?",
      body: h("div", {},
        h("p", {}, "All six drives enter closed loop. ",
          req.controller === "none" ? "With no controller they hold zero torque." :
            `${req.controller} starts and engages once the robot is upright and still.`),
        h("p", {}, "Lift or hold the robot, and keep the power cut-off in reach.")),
      ok: "Arm and start", danger: true, check: "The robot is secured and the cut-off is in reach",
    });
    if (!ok) return;
  }
  await ctx.act("launch/start", req, "launch started").catch(() => {});
}

async function stop() {
  const ok = await ctx.confirm({
    title: "Stop the stack?",
    body: h("p", {}, "One SIGINT, as Ctrl-C: the controller manager sets zero torque, idles the drives and disables the deadman before it exits. A balancing robot falls."),
    ok: "Stop", danger: true,
  });
  if (ok) await ctx.act("launch/stop", {}, "stopping").catch(() => {});
}

function renderLaunch(launch) {
  const key = JSON.stringify([launch.state, launch.run?.pid, launch.external?.length, launch.last_exit?.at]);
  const running = ["running", "starting", "stopping", "external"].includes(launch.state);
  $("#start-form").hidden = running;
  $("#launch-start").disabled = running;
  $("#launch-stop").disabled = !running || launch.state === "stopping";
  $("#launch-stop").textContent = launch.state === "stopping" ? "Stopping…" : "Stop";
  const run = launch.run;
  const level = { running: "ok", stopping: "warn", external: "warn" }[launch.state] || "off";
  replace($("#launch-status"),
    h("div", { class: "status-line" }, dot(level), h("strong", {}, launch.state),
      run ? h("span", { class: "muted" }, ` · up ${duration(run.uptime_s)} · pid ${run.pid}`) : null),
    run ? h("pre", { class: "command" }, run.command) : null,
    run ? h("a", { href: `/api/runs/${run.log}`, class: "small-link" }, run.log) : null,
    launch.state === "external" ? h("p", { class: "hint" },
      "Started outside the panel (deploy/pi/pi launch?). Its output is not captured here; Stop sends it SIGINT.",
      h("pre", { class: "command" }, launch.external[0]?.command || "")) : null,
    !running && launch.last_exit ? h("p", { class: launch.last_exit.code ? "warn-text" : "muted" },
      `Last run exited ${launch.last_exit.code === null ? "" : `with code ${launch.last_exit.code} `}at ${clock(launch.last_exit.at)} · `,
      h("a", { href: `/api/runs/${launch.last_exit.log}`, class: "small-link" }, launch.last_exit.log)) : null);
  if (key !== lastLaunch && !running) renderForm();
  lastLaunch = key;
}

// Controllers ------------------------------------------------------------

const short = (type) => (type || "").split("/").pop();

async function activate(name) {
  if (!simulation()) {
    const ok = await ctx.confirm({
      title: `Start ${name}?`,
      body: h("p", {}, "Hold the robot upright: the legs stand at once and balancing engages when it is upright and still."),
      ok: "Start", danger: true,
    });
    if (!ok) return;
  }
  try {
    await ctx.act("controller/activate", { name }, `${name} active`);
  } catch (error) {
    if (error.status !== 409) return;
    const swap = await ctx.confirm({
      title: `Swap to ${name}?`,
      body: h("div", {},
        h("p", {}, `${error.data.conflict.join(", ")} owns the drives. Swapping hands over in one controller switch, from whatever pose the robot is in.`),
        h("p", {}, "Between the RL policies the gamepad's L1 + Triangle is the soft way: it settles the robot first.")),
      ok: "Swap now", danger: true,
    });
    if (swap) await ctx.act("controller/activate", { name, swap: true }, `${name} active`).catch(() => {});
  }
}

function controllerRow(name, c, manager) {
  const state = c?.state || "not loaded";
  const level = { active: "ok", inactive: "", unconfigured: "warn" }[state] ?? "off";
  const broadcaster = (c?.type || "").includes("broadcaster");
  const button = (label, onclick, cls = "") => h("button", { class: `small ${cls}`, disabled: !manager, onclick }, label);
  return h("tr", { class: broadcaster ? "dim-row" : "" },
    h("td", {}, h("span", { class: "mono" }, name)),
    h("td", { class: "muted" }, short(c?.type)),
    h("td", {}, badge(state, level)),
    h("td", { class: "right actions" }, broadcaster ? null : state === "active"
      ? button("Stop", () => ctx.act("controller/deactivate", { name }).catch(() => {}))
      : [button("Start", () => activate(name)),
        button(state === "inactive" ? "Re-configure" : "Load", () => ctx.act("controller/configure", { name }).catch(() => {}))]));
}

function renderControllers(ros) {
  const manager = !!ros?.manager;
  const listed = Object.fromEntries((ros?.controllers || []).map((c) => [c.name, c]));
  const configured = ctx.state.info.controllers;
  const names = [...configured, ...Object.keys(listed).filter((n) => !configured.includes(n))];
  names.sort((a, b) => (listed[a]?.type || "").includes("broadcaster") - (listed[b]?.type || "").includes("broadcaster"));
  replace($("#controllers"),
    manager ? null : h("p", { class: "hint" }, "No controller manager: start the stack armed (or the simulation)."),
    h("table", { class: "table" }, h("tbody", {}, names.map((n) => controllerRow(n, listed[n], manager)))));
  const hardware = ros?.hardware || [];
  replace($("#hardware"), hardware.length ? h("table", { class: "table" }, h("tbody", {}, hardware.map((hw) => {
    const active = hw.state === "active";
    const arm = async () => {
      const ok = await ctx.confirm({ title: `Arm ${hw.name}?`, body: h("p", {}, "Clears an E-stop latch and puts the drives in closed loop at zero torque, with no controller. Re-arm step 1 of the gamepad does the same."), ok: "Arm", danger: true });
      if (ok) ctx.act("hardware/activate", { name: hw.name }).catch(() => {});
    };
    return h("tr", {},
      h("td", {}, h("span", { class: "mono" }, hw.name)),
      h("td", { class: "muted" }, hw.plugin?.split("/").pop()),
      h("td", {}, badge(hw.state, active ? "ok" : "warn")),
      h("td", { class: "right actions" }, hw.type === "system"
        ? (active ? h("button", { class: "small", onclick: () => ctx.act("hardware/deactivate", { name: hw.name }).catch(() => {}) }, "Disarm")
          : h("button", { class: "small primary", onclick: arm }, "Arm"))
        : null));
  }))) : h("p", { class: "hint" }, "Hardware components appear once the controller manager runs."));

  const diagnostics = ros?.diagnostics || {};
  const active = (ros?.controllers || []).find((c) => c.owns_commands)?.name;
  const shown = active && diagnostics[active] ? active : Object.keys(diagnostics)[0];
  const diag = shown && diagnostics[shown];
  replace($("#diagnostics"), diag ? [
    h("div", { class: "status-line" }, h("span", { class: "mono" }, shown), " · ", h("strong", {}, diag.message || "—")),
    h("div", { class: "kv" }, Object.entries(diag.values).map(([k, v]) =>
      h("div", { class: "kv-item" }, h("span", { class: "kv-key" }, k), h("span", { class: "kv-value mono" }, typeof v === "number" ? num(v, Math.abs(v) >= 100 ? 1 : 3) : v)))),
  ] : h("p", { class: "hint" }, "Live values appear here while a controller publishes ~/diagnostics."));
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
  $("#clear-errors").addEventListener("click", () => ctx.act("clear_errors", {}).catch(() => {}));
  const level = $("#log-level");
  level.value = filter.level;
  level.addEventListener("change", () => { filter.level = level.value; renderLog(); });
  $("#log-filter").addEventListener("input", (e) => { filter.text = e.target.value; renderLog(); });
  $("#log-follow").addEventListener("change", (e) => { filter.follow = e.target.checked; if (filter.follow) renderLog(); });
  $("#runs-toggle").addEventListener("toggle", (e) => { if (e.target.open) runs().catch(() => {}); });
  renderForm();
  ctx.on("slow", (slow) => {
    renderLaunch(slow.launch);
    renderControllers(slow.ros);
  });
  ctx.on("log", appendLog);
}

export function show() {
  renderLog();
}

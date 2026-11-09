// Gamepad: Bluetooth pairing, and the pad's live input as the robot sees it.

import { $, h, replace, badge, num, signed, age } from "./util.js";

let ctx, live;
const NS = "http://www.w3.org/2000/svg";

function s(tag, attrs = {}, ...children) {
  const el = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  el.append(...children);
  return el;
}

// A DualSense outline with every input addressable by its name.
export function padSvg() {
  const parts = {};
  const add = (name, el) => { el.classList.add("key"); parts[name] = el; return el; };
  const svg = s("svg", { viewBox: "0 0 400 250", class: "pad" });
  svg.append(s("path", { class: "pad-body", d: "M120 52 C150 44 250 44 280 52 C320 58 352 74 362 110 C374 150 388 205 368 232 C352 252 326 246 312 226 C296 204 282 188 262 186 L138 186 C118 188 104 204 88 226 C74 246 48 252 32 232 C12 205 26 150 38 110 C48 74 80 58 120 52 Z" }));
  svg.append(s("rect", { class: "pad-touch", x: 150, y: 52, width: 100, height: 58, rx: 9 }));
  const trigger = (name, x) => {
    const g = s("g");
    const frame = add(name, s("rect", { x, y: 4, width: 70, height: 17, rx: 6 }));
    const fill = s("rect", { class: "trigger-fill", x: x + 2, y: 6, width: 0, height: 13, rx: 4 });
    g.append(frame, fill, s("text", { x: x + 35, y: 16.5, class: "key-text" }, name.toUpperCase()));
    svg.append(g);
    return fill;
  };
  const fills = { l2: trigger("l2", 62), r2: trigger("r2", 268) };
  for (const [name, x] of [["l1", 62], ["r1", 268]]) {
    svg.append(add(name, s("rect", { x, y: 26, width: 70, height: 12, rx: 5 })),
      s("text", { x: x + 35, y: 35, class: "key-text small" }, name.toUpperCase()));
  }
  svg.append(add("create", s("rect", { x: 128, y: 56, width: 11, height: 19, rx: 5 })));
  svg.append(add("options", s("rect", { x: 261, y: 56, width: 11, height: 19, rx: 5 })));
  for (const [name, x, y, w, hh] of [["up", 101, 82, 18, 21], ["down", 101, 113, 18, 21], ["left", 77, 99, 21, 18], ["right", 122, 99, 21, 18]]) {
    svg.append(add(name, s("rect", { x, y, width: w, height: hh, rx: 3 })));
  }
  const face = [["triangle", 290, 84], ["circle", 314, 108], ["cross", 290, 132], ["square", 266, 108]];
  for (const [name, cx, cy] of face) {
    const g = s("g");
    g.append(add(name, s("circle", { cx, cy, r: 11 })));
    const mark = { class: "glyph" };
    if (name === "triangle") g.append(s("path", { ...mark, d: `M${cx} ${cy - 5} L${cx + 5} ${cy + 4} L${cx - 5} ${cy + 4} Z` }));
    if (name === "circle") g.append(s("circle", { ...mark, cx, cy, r: 5 }));
    if (name === "cross") g.append(s("path", { ...mark, d: `M${cx - 4.5} ${cy - 4.5} L${cx + 4.5} ${cy + 4.5} M${cx + 4.5} ${cy - 4.5} L${cx - 4.5} ${cy + 4.5}` }));
    if (name === "square") g.append(s("rect", { ...mark, x: cx - 4.5, y: cy - 4.5, width: 9, height: 9 }));
    svg.append(g);
  }
  const sticks = {};
  for (const [side, cx] of [["l", 155], ["r", 245]]) {
    const base = s("circle", { class: "stick-base", cx, cy: 160, r: 25 });
    parts[`${side}x`] = parts[`${side}y`] = base;
    const knob = add(`${side}3`, s("circle", { cx, cy: 160, r: 15 }));
    svg.append(base, knob);
    sticks[side] = { knob, cx, cy: 160 };
  }
  svg.append(add("ps", s("circle", { cx: 200, cy: 170, r: 8 })));
  svg.append(add("mute", s("rect", { x: 193, y: 186, width: 14, height: 5, rx: 2.5 })));
  return { svg, parts, sticks, fills };
}

export function highlight(pad, keys) {
  for (const [name, el] of Object.entries(pad.parts)) el.classList.toggle("lit", keys.includes(name));
}

function showInput(state) {
  if (!state) {
    highlight(live, []);
    for (const side of ["l", "r"]) live.sticks[side].knob.setAttribute("transform", "");
    for (const fill of Object.values(live.fills)) fill.setAttribute("width", 0);
    return;
  }
  for (const [name, el] of Object.entries(live.parts)) {
    if (name.endsWith("x") || name.endsWith("y")) continue;
    el.classList.toggle("pressed", !!state.buttons[name]);
  }
  for (const side of ["l", "r"]) {
    const x = state[`${side}x`] || 0, y = state[`${side}y`] || 0;
    live.sticks[side].knob.setAttribute("transform", `translate(${(x * 10).toFixed(1)} ${(-y * 10).toFixed(1)})`);
  }
  live.fills.l2.setAttribute("width", Math.max(0, Math.min(1, state.l2)) * 66);
  live.fills.r2.setAttribute("width", Math.max(0, Math.min(1, state.r2)) * 66);
}

function bar(label, value, limit, unit) {
  const fraction = value === null || value === undefined ? 0 : Math.max(-1, Math.min(1, value / limit));
  return h("div", { class: "meter-row" },
    h("span", { class: "meter-label" }, label),
    h("div", { class: "meter" }, h("div", { class: "meter-fill", style: {
      left: fraction < 0 ? `${50 + fraction * 50}%` : "50%", width: `${Math.abs(fraction) * 50}%` } })),
    h("span", { class: "meter-value mono" }, value === null || value === undefined ? "—" : `${signed(value, 2)} ${unit}`));
}

function onFast(fast) {
  const pad = fast.pad || {};
  showInput(pad.state);
  const guide = ctx.state.info.guide;
  const state = pad.state;
  const deadman = state?.buttons?.[guide.deadman];
  const estop = state && guide.estop.every((k) => (state[k] ?? 0) >= guide.estop_fraction);
  const cmd = fast.ros?.cmd_vel;
  replace($("#pad-live-status"),
    h("span", {}, pad.source ? `input from ${pad.source}` : "no input", pad.age_s !== undefined && pad.source ? ` · ${age(pad.age_s)}` : ""),
    deadman ? badge(`${guide.labels[guide.deadman]} held: driving enabled`, "ok") : null,
    estop ? badge("E-stop chord", "error") : null);
  replace($("#pad-command"),
    h("h3", {}, "/cmd_vel from teleop"),
    cmd ? [bar("forward", cmd.forward, 1.0, "m/s"), bar("yaw", cmd.yaw, 3.0, "rad/s"),
      bar("height", cmd.height_rate, 0.15, "m/s"), bar("roll", cmd.roll, 0.3, "rad")]
      : h("p", { class: "hint" }, `Nothing published: teleop is not running, or ${guide.labels[guide.deadman]} is released.`));
}

function deviceRow(d, busy) {
  const act = (verb, label, cls = "") => h("button", { class: `small ${cls}`, disabled: !!busy,
    onclick: () => ctx.act(`bt/${verb}`, { mac: d.mac }).catch(() => {}) }, label);
  const battery = d.battery ? d.battery.replace(/^0x[0-9a-f]+ \((\d+)\)$/i, "$1") : null;
  return h("tr", { class: d.gamepad ? "" : "dim-row" },
    h("td", {}, d.name || h("span", { class: "muted" }, "(no name)"), d.gamepad ? h("span", { class: "muted small" }, " gamepad") : null),
    h("td", { class: "mono muted" }, d.mac),
    h("td", {}, d.connected ? badge("connected", "ok") : null, d.paired ? badge("paired") : null, d.trusted ? badge("trusted") : null),
    h("td", { class: "mono" }, battery ? `${battery}%` : d.rssi ? `${d.rssi} dBm` : ""),
    h("td", { class: "right actions" },
      !d.paired ? act("pair", "Pair", "primary") : null,
      d.paired && !d.connected ? act("connect", "Connect", "primary") : null,
      d.connected ? act("disconnect", "Disconnect") : null,
      d.paired ? act("remove", "Forget") : null));
}

function onSlow(slow) {
  const bt = slow.bt || {};
  const box = $("#bt");
  if (!bt.available) {
    replace(box, h("p", { class: "hint" }, "bluetoothctl is not available to the panel. On the robot, ",
      h("code", {}, "deploy/pi/pi setup"), " installs BlueZ in the image and ", h("code", {}, "deploy/pi/pi reset"),
      " gives the container the host's D-Bus socket."));
    return;
  }
  const blocked = (bt.rfkill || []).some((r) => r.soft);
  const hard = (bt.rfkill || []).some((r) => r.hard);
  const busy = bt.busy;
  replace(box,
    h("div", { class: "status-line" },
      badge(bt.powered ? "radio on" : "radio off", bt.powered ? "ok" : "error"),
      blocked ? badge("rfkill soft-blocked", "warn") : null, hard ? badge("rfkill hard-blocked", "error") : null,
      bt.scanning ? badge(`scanning ${Math.ceil(bt.scan_left_s || 0)} s`, "info") : null,
      busy ? badge(busy, "info") : null,
      h("span", { class: "muted mono" }, bt.controller || "")),
    bt.error ? h("p", { class: "warn-text" }, bt.error) : null,
    h("div", { class: "button-row" },
      bt.scanning ? h("button", { onclick: () => ctx.act("bt/stop_scan", {}, false).catch(() => {}) }, "Stop scan")
        : h("button", { class: "primary", disabled: !bt.powered, onclick: () => ctx.act("bt/scan", { seconds: 60 }, false).catch(() => {}) }, "Scan 60 s"),
      blocked || !bt.powered ? h("button", { onclick: () => ctx.act("bt/unblock", {}).catch(() => {}) }, "Unblock and power on") : null),
    h("ol", { class: "steps" },
      h("li", {}, "Hold ", h("kbd", {}, "PS"), " + ", h("kbd", {}, "Create"), " until the light bar flashes fast."),
      h("li", {}, "Scan, then press Pair on ", h("em", {}, "Wireless Controller"), " while the scan still runs."),
      h("li", {}, "Paired once, a press of ", h("kbd", {}, "PS"), " reconnects it.")),
    h("table", { class: "table" }, h("tbody", {}, (bt.devices || []).length
      ? bt.devices.map((d) => deviceRow(d, busy))
      : h("tr", {}, h("td", { class: "empty" }, "No devices yet.")))),
    (bt.output || []).length ? h("pre", { class: "console" }, bt.output.slice(-12).join("\n")) : null);
  const battery = slow.pad?.battery;
  $("#pad-battery").textContent = battery ? `battery ${battery.percent}% · ${battery.status.toLowerCase()}` : "";
  $("#pad-device").textContent = slow.pad?.device ? `${slow.pad.name || "joystick"} on ${slow.pad.device}` : "no joystick device";
}

export function init(context) {
  ctx = context;
  live = padSvg();
  $("#pad-live").append(live.svg);
  ctx.on("fast", onFast);
  ctx.on("slow", onSlow);
}

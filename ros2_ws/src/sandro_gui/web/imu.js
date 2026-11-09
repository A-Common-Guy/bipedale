// IMU: attitude, rates, acceleration, how fresh the samples are, and strip charts.

import { $, h, replace, num, signed, deg, hz, age, euler } from "./util.js";

let ctx;
let paused = false;
let visible = false;
const charts = [];
const COLORS = ["#e5534b", "#57ab5a", "#539bf5", "#c69026"];

class Strip {
  constructor(canvas, series, unit, minSpan) {
    this.canvas = canvas;
    this.series = series;
    this.unit = unit;
    this.minSpan = minSpan;
    this.span = 15;
    this.points = [];
  }

  push(t, values) {
    this.points.push([t, values]);
    while (this.points.length && t - this.points[0][0] > this.span) this.points.shift();
  }

  draw() {
    const c = this.canvas;
    const ratio = window.devicePixelRatio || 1;
    const w = c.clientWidth, hgt = c.clientHeight;
    if (!w || !hgt) return;
    if (c.width !== Math.round(w * ratio)) { c.width = Math.round(w * ratio); c.height = Math.round(hgt * ratio); }
    const g = c.getContext("2d");
    g.setTransform(ratio, 0, 0, ratio, 0, 0);
    g.clearRect(0, 0, w, hgt);
    const left = 46, right = 8, top = 8, bottom = 16;
    let lo = Infinity, hi = -Infinity;
    for (const [, values] of this.points) for (const v of values) if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
    if (!Number.isFinite(lo)) { lo = -this.minSpan / 2; hi = this.minSpan / 2; }
    const mid = (lo + hi) / 2, span = Math.max(this.minSpan, (hi - lo) * 1.15);
    lo = mid - span / 2; hi = mid + span / 2;
    const y = (v) => top + (hgt - top - bottom) * (1 - (v - lo) / (hi - lo));
    const now = this.points.length ? this.points[this.points.length - 1][0] : 0;
    const x = (t) => left + (w - left - right) * (1 - (now - t) / this.span);
    g.font = "11px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    g.fillStyle = "#7d8590";
    g.strokeStyle = "#262b31";
    g.lineWidth = 1;
    for (let k = 0; k <= 4; k++) {
      const v = lo + ((hi - lo) * k) / 4;
      const py = Math.round(y(v)) + 0.5;
      g.beginPath(); g.moveTo(left, py); g.lineTo(w - right, py); g.stroke();
      g.fillText(v.toFixed(Math.abs(hi - lo) < 2 ? 2 : 1), 4, py + 4);
    }
    if (lo < 0 && hi > 0) {
      g.strokeStyle = "#3a4149";
      const py = Math.round(y(0)) + 0.5;
      g.beginPath(); g.moveTo(left, py); g.lineTo(w - right, py); g.stroke();
    }
    for (let s = 0; s < this.series.length; s++) {
      g.strokeStyle = COLORS[s];
      g.lineWidth = 1.5;
      g.beginPath();
      let pen = false;
      for (const [t, values] of this.points) {
        const v = values[s];
        if (!Number.isFinite(v)) { pen = false; continue; }
        if (pen) g.lineTo(x(t), y(v)); else g.moveTo(x(t), y(v));
        pen = true;
      }
      g.stroke();
    }
    g.fillText(`${this.span} s`, w - right - 30, hgt - 3);
  }
}

function chart(id, title, series, unit, minSpan) {
  const canvas = h("canvas", { class: "strip" });
  const legend = h("div", { class: "legend" }, series.map((name, i) =>
    h("span", {}, h("i", { style: { background: COLORS[i] } }), name)), h("span", { class: "muted" }, unit));
  replace($(id), h("div", { class: "panel-head" }, h("h2", {}, title), legend), canvas);
  const strip = new Strip(canvas, series, unit, minSpan);
  charts.push(strip);
  return strip;
}

let attitude, gyro, accel;

function cell(label, value, unit = "") {
  return h("div", { class: "reading" }, h("span", { class: "reading-label" }, label),
    h("span", { class: "reading-value" }, value), h("span", { class: "reading-unit" }, unit));
}

function onFast(fast) {
  const ros = fast.ros || {};
  const imu = ros.imu;
  const rate = ros.rates?.["/imu_broadcaster/imu"] || {};
  const live = imu && ros.imu_age_s !== null && ros.imu_age_s < 0.5;
  const t = performance.now() / 1000;
  if (live && !paused) {
    const e = euler(imu.q);
    attitude.push(t, [e.roll * 57.2958, e.pitch * 57.2958]);
    gyro.push(t, imu.gyro);
    accel.push(t, imu.accel);
  }
  if (!visible) return;
  if (!paused) for (const c of charts) c.draw();
  if (!imu) {
    replace($("#imu-readings"), h("p", { class: "empty" },
      "No IMU data. It is published while the stack runs armed, in simulation, or as the IMU check (Run page)."));
    return;
  }
  const e = euler(imu.q);
  const [ax, ay, az] = imu.accel;
  const norm = Math.hypot(ax, ay, az);
  const tilt = Math.atan2(Math.hypot(ax, ay), az);
  const dyn = ctx.state.slow?.ros?.dynamic?.imu || {};
  const model = ctx.state.model;
  const chipYaw = model ? (2 * Math.atan2(model.imu_chip_quat[3], model.imu_chip_quat[0]) * 180) / Math.PI : null;
  replace($("#imu-readings"),
    h("div", { class: "panel reading-group" }, h("h3", {}, "Attitude"),
      cell("roll", signed(e.roll * 57.2958, 2), "°"), cell("pitch", signed(e.pitch * 57.2958, 2), "°"),
      cell("yaw", signed(e.yaw * 57.2958, 1), "°"), cell("tilt from gravity", num(tilt * 57.2958, 2), "°")),
    h("div", { class: "panel reading-group" }, h("h3", {}, "Angular rate"),
      cell("x", signed(imu.gyro[0], 3), "rad/s"), cell("y", signed(imu.gyro[1], 3), "rad/s"),
      cell("z", signed(imu.gyro[2], 3), "rad/s")),
    h("div", { class: "panel reading-group" }, h("h3", {}, "Acceleration"),
      cell("x", signed(ax, 2), "m/s²"), cell("y", signed(ay, 2), "m/s²"), cell("z", signed(az, 2), "m/s²"),
      cell("|a|", num(norm, 2), "m/s²")),
    h("div", { class: `panel reading-group ${live ? "" : "stale"}` }, h("h3", {}, "Stream"),
      cell("topic", hz(rate.hz), "Hz"),
      cell("sample age", ros.imu_sensor_age_s === null || ros.imu_sensor_age_s === undefined ? "—" : age(ros.imu_sensor_age_s), "driver"),
      cell("message age", age(ros.imu_age_s)), cell("driver", dyn.online === undefined ? "—" : dyn.online > 0.5 ? "online" : "OFFLINE"),
      chipYaw === null ? null : cell("chip in holder", `${num(chipYaw, 0)}°`, "yaw")));
}

export function init(context) {
  ctx = context;
  attitude = chart("#chart-attitude", "Roll and pitch", ["roll", "pitch"], "deg", 2);
  gyro = chart("#chart-gyro", "Angular rate", ["x", "y", "z"], "rad/s", 0.2);
  accel = chart("#chart-accel", "Acceleration", ["x", "y", "z"], "m/s²", 2);
  $("#imu-pause").addEventListener("click", (e) => {
    paused = !paused;
    e.target.textContent = paused ? "Resume" : "Pause";
  });
  ctx.on("fast", onFast);
}

export function show() {
  visible = true;
}

export function hide() {
  visible = false;
}

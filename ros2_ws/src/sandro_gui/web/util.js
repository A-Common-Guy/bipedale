// DOM and formatting helpers shared by every page.

export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs || {})) {
    if (value === undefined || value === null || value === false) continue;
    if (key === "class") el.className = value;
    else if (key === "style" && typeof value === "object") Object.assign(el.style, value);
    else if (key.startsWith("on")) el.addEventListener(key.slice(2), value);
    else if (key === "html") el.innerHTML = value;
    else el.setAttribute(key, value === true ? "" : value);
  }
  for (const child of children.flat(Infinity)) {
    if (child === undefined || child === null || child === false) continue;
    el.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
  return el;
}

export const $ = (selector, root = document) => root.querySelector(selector);

export function replace(el, ...children) {
  el.replaceChildren(...children.flat(Infinity).filter((c) => c !== null && c !== undefined && c !== false));
  return el;
}

export function num(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) return "—";
  return Number(value).toFixed(digits);
}

export function signed(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) return "—";
  const text = Number(value).toFixed(digits);
  return value >= 0 ? "+" + text : text;
}

export const deg = (rad, digits = 1) => (rad === null || rad === undefined ? "—" : num((rad * 180) / Math.PI, digits));

export function hz(value) {
  if (value === null || value === undefined) return "—";
  return value >= 100 ? value.toFixed(0) : value.toFixed(1);
}

export function age(seconds) {
  if (seconds === null || seconds === undefined) return "—";
  if (seconds < 1) return `${(seconds * 1000).toFixed(0)} ms`;
  if (seconds < 120) return `${seconds.toFixed(1)} s`;
  return duration(seconds);
}

export function duration(seconds) {
  if (seconds === null || seconds === undefined) return "—";
  const s = Math.floor(seconds);
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, "0")} s`;
  return `${Math.floor(s / 3600)} h ${String(Math.floor(s / 60) % 60).padStart(2, "0")} min`;
}

export function bytes(n) {
  if (n === null || n === undefined) return "—";
  if (n < 1024) return `${n} B`;
  if (n < 1024 ** 2) return `${(n / 1024).toFixed(0)} kB`;
  if (n < 1024 ** 3) return `${(n / 1024 ** 2).toFixed(1)} MB`;
  return `${(n / 1024 ** 3).toFixed(1)} GB`;
}

export function clock(t) {
  const d = new Date(t * 1000);
  return d.toTimeString().slice(0, 8);
}

export const dot = (level) => h("span", { class: `dot ${level || "off"}` });

export function badge(text, level) {
  return h("span", { class: `badge ${level || ""}` }, text);
}

// Roll, pitch, yaw (rad, ZYX) of an (x, y, z, w) quaternion.
export function euler(q) {
  const [x, y, z, w] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sinp);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

export const LEG = { base: 0.09, upper: 0.08, lower: 0.13 };

// Axle in the five-bar plane (x inner->outer pivot, y down), as sandro_controllers/kinematics.cpp.
export function fiveBar(thetaInner, thetaOuter, g = LEG) {
  const p = g.upper * (Math.cos(thetaOuter) - Math.cos(thetaInner));
  const q = g.base + g.upper * (Math.sin(thetaInner) + Math.sin(thetaOuter));
  const r = Math.hypot(p, q);
  if (!(r <= 2 * g.lower)) return null;
  const b1 = Math.acos(Math.min(1, r / (2 * g.lower))) + Math.atan2(p, q);
  return {
    x: -g.upper * Math.sin(thetaInner) + g.lower * Math.cos(b1),
    y: g.upper * Math.cos(thetaInner) + g.lower * Math.sin(b1),
    kneeInner: { x: -g.upper * Math.sin(thetaInner), y: g.upper * Math.cos(thetaInner) },
    kneeOuter: { x: g.base + g.upper * Math.sin(thetaOuter), y: g.upper * Math.cos(thetaOuter) },
  };
}

export const JOINTS = ["right_motor_inner", "right_motor_outer", "right_wheel",
  "left_motor_inner", "left_motor_outer", "left_wheel"];

export const SHORT = {
  right_motor_inner: "R inner", right_motor_outer: "R outer", right_wheel: "R wheel",
  left_motor_inner: "L inner", left_motor_outer: "L outer", left_wheel: "L wheel",
};

export function download(name, text, type = "text/plain") {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = h("a", { href: url, download: name });
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

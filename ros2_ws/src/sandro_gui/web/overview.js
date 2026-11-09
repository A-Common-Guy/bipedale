// Overview: the robot in 3D, the pre-flight checks, every drive, and what went wrong.

import { $, h, replace, dot, badge, num, deg, hz, age, clock, euler, JOINTS, SHORT } from "./util.js";
import { RobotView } from "./robot3d.js";

let ctx, view;
let source = "none";

function jointSource(fast) {
  const ros = fast?.ros;
  if (ros?.joints && ros.joints_age_s !== null && ros.joints_age_s < 0.5 && Object.keys(ros.joints).length) {
    return { kind: "/joint_states", joints: Object.fromEntries(Object.entries(ros.joints).map(([n, j]) => [n, j.position])) };
  }
  const can = fast?.can_joints || {};
  if (Object.keys(can).length) {
    return { kind: "CAN encoder broadcast", joints: Object.fromEntries(Object.entries(can).map(([n, j]) => [n, j.position])) };
  }
  return { kind: null, joints: {} };
}

function onFast(fast) {
  if (!view) return;
  const { kind, joints } = jointSource(fast);
  const imu = fast.ros?.imu && fast.ros.imu_age_s !== null && fast.ros.imu_age_s < 0.5 ? fast.ros.imu : null;
  view.update(joints, imu ? imu.q : null);
  if (kind !== source) {
    source = kind;
    $("#viewer-note").textContent = kind ? `joints from ${kind}` : "no joint data: drawn in the stand pose";
    $("#viewer-note").classList.toggle("warn", !kind);
  }
  const legs = view.legState();
  const att = imu ? euler(imu.q) : null;
  const rows = [
    ["R inner", deg(joints.right_motor_inner)], ["R outer", deg(joints.right_motor_outer)],
    ["L inner", deg(joints.left_motor_inner)], ["L outer", deg(joints.left_motor_outer)],
  ];
  replace($("#pose-overlay"),
    h("table", {},
      h("tbody", {},
        rows.map(([label, value]) => h("tr", {}, h("th", {}, label), h("td", {}, kind ? `${value}°` : "—"))),
        h("tr", { class: "sep" }, h("th", {}, "height R/L"), h("td", {}, legs.right && kind ? `${num(legs.right.height * 1000, 0)} / ${num(legs.left.height * 1000, 0)} mm` : "—")),
        h("tr", {}, h("th", {}, "axle fwd R/L"), h("td", {}, legs.right && kind ? `${num(legs.right.forward * 1000, 0)} / ${num(legs.left.forward * 1000, 0)} mm` : "—")),
        h("tr", { class: "sep" }, h("th", {}, "pitch"), h("td", {}, att ? `${deg(att.pitch)}°` : "—")),
        h("tr", {}, h("th", {}, "roll"), h("td", {}, att ? `${deg(att.roll)}°` : "—")),
        h("tr", { title: "Body roll the two leg heights give with both wheels on level ground" },
          h("th", {}, "roll from legs"), h("td", {}, kind && legs.roll !== undefined ? `${deg(legs.roll)}°` : "—")),
        h("tr", { title: "With both wheels on level ground: the IMU's roll mounting offset" },
          h("th", {}, "IMU − legs"), h("td", {}, att && kind && legs.roll !== undefined ? `${deg(att.roll - legs.roll)}°` : "—")))));
}

function onSlow(slow) {
  replace($("#checks"), (slow.checks || []).map((check) =>
    h("li", { class: `check ${check.level}` },
      dot(check.level),
      h("div", { class: "check-main" },
        h("div", { class: "check-line" }, h("span", { class: "check-label" }, check.label), h("span", { class: "check-value" }, check.value)),
        check.detail ? h("div", { class: "check-detail" }, check.detail) : null))));
  drives(slow);
  streams(slow);
  problems(slow.problems || []);
}

function streams(slow) {
  const ros = ctx.state.fast?.ros || {};
  const rates = ros.rates || {};
  const ages = Object.values(ros.joints || {}).map((j) => j.sensor_age_s).filter((v) => v !== null && v !== undefined);
  const sensorAge = { "/joint_states": ages.length ? Math.max(...ages) : null, "/imu_broadcaster/imu": ros.imu_sensor_age_s };
  const running = ["running", "external"].includes(slow.launch?.state);
  const deadman = ctx.state.fast?.pad?.state?.buttons?.[ctx.state.info.guide.deadman];
  const rows = (ctx.state.info.topics || []).map((topic) => {
    const r = rates[topic.name] || {};
    const fresh = r.age_s !== null && r.age_s !== undefined && r.age_s < 1;
    const gated = topic.while === "deadman";
    let level = "off";
    if (fresh) level = !topic.expect_hz || r.hz >= 0.8 * topic.expect_hz ? "ok" : "warn";
    else if (running && slow.ros?.manager && !(gated && !deadman) && topic.name !== "/joy") level = "error";
    const note = gated ? `only while ${ctx.state.info.guide.labels[ctx.state.info.guide.deadman]} is held` : "";
    return h("tr", {},
      h("td", {}, dot(level), " ", h("span", { class: "mono" }, topic.name)),
      h("td", { class: "muted" }, topic.type.split("/").pop()),
      h("td", { class: "mono" }, fresh ? `${hz(r.hz)} Hz` : "—"),
      h("td", { class: "mono muted" }, topic.expect_hz ? `${topic.expect_hz} Hz` : ""),
      h("td", { class: "mono" }, r.age_s === null || r.age_s === undefined ? "never" : age(r.age_s)),
      h("td", { class: "mono" }, fresh && sensorAge[topic.name] !== null && sensorAge[topic.name] !== undefined ? age(sensorAge[topic.name]) : ""),
      h("td", { class: "muted" }, note));
  });
  const link = slow.can_link;
  if (link?.present) {
    rows.push(h("tr", {},
      h("td", {}, dot(link.rx_hz > 0 ? "ok" : link.up ? "warn" : "error"), " ", h("span", { class: "mono" }, slow.can?.interface || "can0")),
      h("td", { class: "muted" }, "CAN frames"),
      h("td", { class: "mono" }, link.rx_hz === undefined ? "—" : `${hz(link.rx_hz)} rx · ${hz(link.tx_hz)} tx`),
      h("td", {}), h("td", {}), h("td", {}),
      h("td", { class: "muted" }, "the whole bus, from the interface counters")));
  }
  replace($("#streams"),
    h("thead", {}, h("tr", {}, ["Stream", "Type", "Rate", "Expected", "Last message", "Sample age", ""].map((t) => h("th", {}, t)))),
    h("tbody", {}, rows));
}

function drives(slow) {
  const fast = ctx.state.fast || {};
  const ros = fast.ros || {};
  const rosJoints = ros.joints_age_s !== null && ros.joints_age_s < 0.5 ? ros.joints || {} : {};
  const canNodes = Object.fromEntries((slow.can?.nodes || []).filter((n) => n.joint).map((n) => [n.joint, n]));
  const dynamic = slow.ros?.dynamic || {};
  $("#drives-source").textContent = slow.can ? `bus: ${slow.can.interface}${slow.can.open ? "" : " (not open)"}` : "";
  const head = h("thead", {}, h("tr", {},
    ["Joint", "Node", "State", "Position", "Velocity", "Effort", "Temp", "Heartbeat", "Encoder", "Sample age", "Error"].map((t) => h("th", {}, t))));
  const body = h("tbody", {}, JOINTS.map((name) => {
    const ros = rosJoints[name];
    const can = canNodes[name] || {};
    const dyn = dynamic[name] || {};
    const position = ros ? ros.position : can.position;
    const velocity = ros ? ros.velocity : can.velocity;
    const heard = can.heartbeat_age_s !== null && can.heartbeat_age_s !== undefined && can.heartbeat_age_s < 1;
    const online = dyn.online !== undefined ? dyn.online > 0.5 : heard;
    const error = can.error_text || (dyn.error > 0 ? "drive error" : "");
    const level = error ? "warn" : online ? "ok" : (slow.can || dyn.online !== undefined ? "error" : "off");
    const rates = can.rates || {};
    const temp = can.temperature ?? dyn.temperature;
    return h("tr", { class: level === "error" ? "row-error" : "" },
      h("td", {}, dot(level), " ", SHORT[name]),
      h("td", { class: "mono" }, can.node ?? "—"),
      h("td", {}, can.axis_state ? badge(can.axis_state, can.axis_state === "closed loop" ? "ok" : "") : "—"),
      h("td", { class: "mono" }, position === undefined ? "—" : `${num(position, 3)} rad`),
      h("td", { class: "mono" }, velocity === undefined ? "—" : num(velocity, 2)),
      h("td", { class: "mono" }, ros ? `${num(ros.effort, 2)} Nm` : "—"),
      h("td", { class: "mono" }, temp === undefined || temp === null ? "—" : `${num(temp, 0)} °C`),
      h("td", { class: "mono" }, can.heartbeat_age_s === undefined ? "—" : heard ? `${hz(rates.heartbeat || 0)} Hz`
        : can.heartbeat_age_s === null ? "never heard" : `silent ${age(can.heartbeat_age_s)}`),
      h("td", { class: "mono" }, rates.encoder ? `${hz(rates.encoder)} Hz` : "—"),
      h("td", { class: "mono" }, ros?.sensor_age_s === null || ros?.sensor_age_s === undefined ? "—" : age(ros.sensor_age_s)),
      h("td", { class: error ? "warn-text" : "muted" }, error || (dyn.saturation_count ? `${dyn.saturation_count} saturations` : "")));
  }));
  replace($("#drives"), head, body);
}

function problems(list) {
  const box = $("#problems");
  if (!list.length) {
    replace(box, h("p", { class: "empty" }, "No warnings or errors."));
    return;
  }
  replace(box, h("table", { class: "problems" }, h("tbody", {}, list.map((p) =>
    h("tr", { class: p.level },
      h("td", { class: "mono muted" }, clock(p.last)),
      h("td", {}, badge(p.level, p.level === "warn" ? "warn" : "error")),
      h("td", { class: "mono muted" }, p.source),
      h("td", { class: "problem-text" }, p.text),
      h("td", { class: "mono muted right" }, p.count > 1 ? `×${p.count}` : ""))))));
}

export function init(context) {
  ctx = context;
  const tools = $("#viewer-tools");
  const toggle = (label, key, initial) => {
    const input = h("input", { type: "checkbox", checked: initial });
    input.addEventListener("change", () => view?.setOptions({ [key]: input.checked }));
    return h("label", { class: "check small" }, input, label);
  };
  replace(tools,
    toggle("IMU attitude", "attitude", true), toggle("Frames", "frames", true), toggle("Links", "skeleton", false),
    h("button", { class: "small", onclick: () => view?.resetView() }, "Reset view"));
  try {
    view = new RobotView($("#viewer"));
    view.load().then((meta) => { ctx.state.model = meta; }).catch((error) => {
      $("#viewer-note").textContent = `3D model: ${error.message}`;
      $("#viewer-note").classList.add("warn");
      view.setOptions({ skeleton: true });
    });
  } catch (error) {
    $("#viewer").append(h("p", { class: "empty" }, `WebGL unavailable: ${error.message}`));
  }
  $("#problems-clear").addEventListener("click", () => ctx.act("problems/clear", {}, false));
  ctx.on("fast", onFast);
  ctx.on("slow", onSlow);
}

export function show() {
  view?.setVisible(true);
}

export function hide() {
  view?.setVisible(false);
}

// Guide: the start-up procedure and every gamepad combination, from teleop.yaml.

import { $, h, replace } from "./util.js";
import { padSvg, highlight } from "./pad.js";

const STEPS = [
  ["Power and secure", "Robot lifted or on its stand, drives powered, the power cut-off in reach. The Pi starts the panel by itself: open http://<pi>:8080."],
  ["Bus and drives", "Overview: CAN bus error-active at 1 Mbit/s and Drives 6/6 heard, before anything is launched. A silent drive shows its node and how long it has been quiet. Error 0x104000 is the E-stop latch, which arming clears; any other code must be inspected."],
  ["Gamepad", "Press PS: the Gamepad check turns green with the battery level. A new pad is paired on the Gamepad page."],
  ["IMU, drives off (optional)", "Run, IMU check, Start. The IMU page should read about 240 new samples per second, tilt near 0° on a level floor, and positive pitch when the robot tips forward. Stop it before the next step."],
  ["Arm", "Run, Armed, controller none, gamepad teleop ticked, Start. The drives hold zero torque. Overview: joint states at 300 Hz, the IMU fresh, and the 3D legs following when you move a leg by hand."],
  ["Start the controller", "Hold the robot upright, then Start the controller on the Run page or hold Create + Options for 2 s on the pad. It engages once the robot is upright and still."],
  ["Drive", "Hold L1 and use the sticks (table below). Watch the Run page's diagnostics for saturation."],
  ["Stop", "E-STOP in the header or L2 + R2 for an immediate stop. Run, Stop for a normal shutdown: one SIGINT, the drives go idle and the deadman off before the launch exits."],
];

const TROUBLE = [
  ["A drive is silent", "Its heartbeat age grows in the Drives table. Check that drive's CAN cable and power; the rest of the bus keeps working. Arming refuses while any drive is missing."],
  ["CAN bus-off or error-passive", "Bit rate or sample point mismatch, a missing terminator, or a drive powered down mid-run. can0.service configures 1 Mbit/s at boot; the bus error counters are in the CAN check."],
  ["IMU stale or IMU FAULT in the log", "Run the IMU check with the drives off. Over SPI6: INT on GPIO25, RST on GPIO24, P0 and P1 tied high (deploy/pi/README.md)."],
  ["Arm refused", "The Run log names the drive and its axis error. Only the E-stop and watchdog bits clear on arming; an encoder or temperature fault does not. A MIT frame bias above 0.2 rad means a drive zero is off."],
  ["Gamepad does not connect", "Gamepad page: unblock the radio if rfkill blocks it, scan, and pair while the scan runs. Once paired, PS reconnects it."],
  ["A policy oscillates", "The RL controllers cut the drives on a 5-15 Hz oscillation (RL_OSCILLATION in the log). Re-arm and try again from a calmer start; check the leg zeros if it repeats."],
];

function keys(entry, labels) {
  const chips = entry.keys.map((k) => h("kbd", {}, labels[k] || k));
  const joined = [];
  chips.forEach((chip, i) => { if (i) joined.push(" + "); joined.push(chip); });
  return h("span", { class: "keys" }, entry.hold ? h("span", { class: "muted" }, "hold ") : null, joined);
}

export function init(ctx) {
  const guide = ctx.state.info.guide;
  const pad = padSvg();
  $("#guide-pad").append(pad.svg);
  const groups = new Map();
  for (const entry of guide.entries) {
    if (!groups.has(entry.group)) groups.set(entry.group, []);
    groups.get(entry.group).push(entry);
  }
  replace($("#guide-combos"), [...groups].map(([group, entries]) => [
    h("h4", {}, group),
    h("table", { class: "combo-table" }, h("tbody", {}, entries.map((entry) =>
      h("tr", { onmouseenter: () => highlight(pad, entry.keys), onmouseleave: () => highlight(pad, []) },
        h("td", { class: "combo-keys" }, keys(entry, guide.labels)),
        h("td", {}, entry.text))))),
  ]));
  replace($("#guide-lights"),
    h("h4", {}, "Light bar"),
    h("ul", { class: "legend-list" }, guide.lightbar.map((item) =>
      h("li", {}, h("i", { class: "swatch", style: { background: `rgb(${item.rgb.join(",")})` } }), item.policy))),
    h("p", {}, "During a policy switch the light bar blinks between the two colours. The player LEDs light up to the running policy's position in the list."),
    h("h4", {}, "Rumble"),
    h("p", {}, "Pulses count the running policy: one for the first, two for the second. It plays whenever the running controller changes and on Circle. One long rumble: a switch was refused or aborted."),
    h("h4", {}, "When teleop runs"),
    h("p", {}, "With Gamepad teleop ticked on the Run page, and on the real robot only once the drives are armed. Full stick deflection is the running controller's own limit, read from its parameters, so it means the same on every policy."));
  replace($("#guide-steps"), h("ol", { class: "steps long" }, STEPS.map(([title, text]) =>
    h("li", {}, h("strong", {}, title), h("span", {}, text)))));
  replace($("#guide-trouble"), h("table", { class: "combo-table" }, h("tbody", {}, TROUBLE.map(([symptom, fix]) =>
    h("tr", {}, h("td", { class: "combo-keys" }, symptom), h("td", {}, fix))))));
  replace($("#guide-estop"),
    h("p", {}, "Both software E-stops (the header button, L2 + R2) deactivate every controller, so all torques go to zero, and send the CAN E-stop. The drives then latch ESTOP_REQUESTED."),
    h("p", {}, "To continue: arm again (Run page, Hardware, Arm, or hold Create + Options for 2 s), then start the controller (Run page, or hold Create + Options again with the robot held upright)."),
    h("p", { class: "warn-text" }, "Software stops ride on the same bus and computer that just misbehaved. The physical cut-off is the real E-stop."));
}

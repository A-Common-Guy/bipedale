// Sysid: the measurements the RL training needs, one button each (sysid.py).
// The panel checks the robot, runs the tool with fixed arguments, shows its
// instruction of the moment and keeps the result with its downloads.

import { $, h, replace, badge, duration, bytes } from "./util.js";

let ctx = null;
let timer = null;
let data = null;

export function init(c) {
  ctx = c;
  $("#sysid-stop").addEventListener("click", stop);
  // The job state rides on the slow snapshot: a start or an end shows at once.
  ctx.on("slow", (slow) => {
    if (ctx.state.page !== "sysid" || !data) return;
    if ((slow.sysid?.state || null) !== (data.job?.state || null)) refresh();
  });
}

export function show() {
  refresh();
  timer = setInterval(refresh, 1000);
}

export function hide() {
  clearInterval(timer);
  timer = null;
}

async function refresh() {
  try {
    data = await ctx.api("sysid");
  } catch (error) {
    replace($("#sysid-cards"), h("p", { class: "hint" }, error.message));
    return;
  }
  render();
}

function running() {
  return data?.job?.state === "running";
}

function render() {
  renderCards();
  renderJob();
  renderRuns();
}

function renderCards() {
  replace($("#sysid-cards"), data.measurements.map((m) => {
    const reason = running() ? "a measurement is running" : m.blocked;
    return h("div", { class: "panel" },
      h("div", { class: "panel-head" }, h("h2", {}, m.title),
        reason ? badge("not ready", "warn") : badge("ready", "ok")),
      h("p", { class: "hint" }, m.purpose),
      h("ol", {}, m.steps.map((step) => h("li", {}, step))),
      h("div", { class: "sysid-start" },
        h("button", { class: "primary", disabled: !!reason, title: reason || "", onclick: () => start(m) },
          `Start (${duration(m.seconds)})`),
        reason ? h("span", { class: "warn-text small" }, reason)
          : h("span", { class: "muted small" }, m.id === "buzz" ? `records ${data.controller}` : "the WBC is balancing")));
  }));
}

function renderJob() {
  const job = data.job;
  const now = $("#sysid-now");
  const bar = $("#sysid-progress").firstElementChild;
  $("#sysid-stop").disabled = !running();
  if (!job) {
    $("#sysid-job").textContent = "nothing has run yet";
    now.className = "sysid-now idle";
    now.textContent = "Start a measurement above. Its instructions appear here.";
    bar.style.width = "0";
    replace($("#sysid-tail"));
    return;
  }
  const took = (job.ended || Date.now() / 1000) - job.started;
  $("#sysid-job").textContent = `${job.title} · ${job.name} · ${job.state} · ${duration(took)}`;
  const last = data.tail[data.tail.length - 1] || "";
  now.className = running() ? "sysid-now" : "sysid-now idle";
  now.textContent = running() ? (data.now || last.trim() || "starting…")
    : job.state === "done" ? "Finished: the result is in the table below."
      : job.state === "stopped" ? "Stopped. Whatever was recorded until then is below."
        : `Ended without finishing (${job.state}): the log below says why.`;
  bar.style.width = running() ? `${Math.min(100, (100 * took) / job.seconds).toFixed(0)}%` : (job.state === "done" ? "100%" : "0");
  const tail = $("#sysid-tail");
  const atBottom = tail.scrollTop + tail.clientHeight >= tail.scrollHeight - 20;
  tail.textContent = data.tail.join("\n");
  if (atBottom) tail.scrollTop = tail.scrollHeight;
}

function when(seconds) {
  return new Date(seconds * 1000).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
}

function link(run, file, label, title) {
  const href = file ? `/api/sysid/runs/${encodeURIComponent(run.name)}/${file}` : `/api/sysid/runs/${encodeURIComponent(run.name)}.tar`;
  return h("a", { class: "button small", href, title, download: "" }, label);
}

// What a result says about itself, and what to do with it.
function verdict(run) {
  const r = run.result || {};
  if (run.kind === "yaw") {
    if (!r.plant) return [badge("no plant table", "warn"), " ", r.text || "the run did not finish"];
    return [r.weak ? badge("weak", "warn") : badge("good", "ok"), " ",
      `clean response up to ${r.usable_hz.toFixed(0)} Hz of ${r.f_hz[1].toFixed(0)}`,
      r.weak ? ": too little for the training, repeat the run on a flat, grippy floor" : "",
      h("div", { class: "muted small" }, r.text)];
  }
  if (!r.text) return [badge("no analysis", "warn"), " the run did not finish"];
  return [r.text.startsWith("quiet") ? badge("quiet", "ok") : badge("buzz", "warn"), " ", r.text];
}

function renderRuns() {
  $("#sysid-dir").textContent = data.dir;
  const box = $("#sysid-runs");
  if (!data.runs.length) {
    replace(box, h("p", { class: "hint" }, "No runs yet."));
    return;
  }
  const active = running() ? data.job.name : null;
  replace(box,
    h("table", { class: "table" },
      h("tr", {}, h("th", {}, "Run"), h("th", {}, "When"), h("th", {}, "Result"), h("th", {}, "")),
      data.runs.map((run) => h("tr", {},
        h("td", {}, h("div", {}, run.title), h("div", { class: "mono muted small" }, run.name)),
        h("td", { class: "sw-when" }, when(run.time)),
        h("td", {}, run.name === active ? badge("running", "info") : verdict(run)),
        h("td", { class: "actions" },
          run.name === active ? null : [
            run.result?.plant ? link(run, "yaw_plant.json", "Plant table", "yaw_plant_<run>.json for sandro_rl/data/") : null,
            run.files.includes("summary.json") ? link(run, "summary.json", "Summary", "the analysis as JSON") : null,
            link(run, null, `Recording (${bytes(run.bytes)})`, "the whole run as a .tar: bag, plan, log, results"),
          ])))),
    h("p", { class: "hint sysid-next" },
      "Plant table: save it into sandro_rl/data/ on the training machine and set train.loop_margin.plant to its file name. ",
      "Recording: untar it under recordings/ to keep it with the others."));
}

async function start(m) {
  const ok = await ctx.confirm({
    title: `${m.title}?`,
    body: h("div", {},
      h("ol", {}, m.steps.map((step) => h("li", {}, step))),
      h("p", { class: "hint" }, m.id === "yaw"
        ? "The whole-body controller adds a small opposed wheel torque while it balances. Stop measurement, the E-stop, or the robot no longer balancing end it at once."
        : "Recording only: the controller is not touched.")),
    ok: "Start",
    check: "I am watching the robot and the E-stop is in reach",
  });
  if (!ok) return;
  await ctx.act("sysid/start", { kind: m.id, watching: true }, `${m.title} started`).catch(() => {});
  refresh();
}

async function stop() {
  await ctx.act("sysid/stop", {}).catch(() => {});
  refresh();
}

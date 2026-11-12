// Software: the version on disk, built and running, and the one-button update
// from the robot's repository (deploy/pi/pi push, or Fetch from GitHub).

import { $, h, replace, badge, duration } from "./util.js";

let ctx = null;
let timer = null;
let data = null;
let ref = null;

function label(v) {
  return v ? (v.short || v.commit.slice(0, 7)) + (v.dirty ? "+" : "") : "—";
}

// software.py's same_version: the same commit, and for uncommitted changes the same sync.
function same(a, b) {
  if (!a || !b || a.commit !== b.commit || !!a.dirty !== !!b.dirty) return false;
  return !a.dirty || a.at === b.at;
}

function when(value) {
  if (!value) return "";
  const t = typeof value === "number" ? value * 1000 : Date.parse(value);
  if (Number.isNaN(t)) return value;
  const ago = (Date.now() - t) / 1000;
  if (ago < 3600) return `${Math.max(Math.round(ago / 60), 0)} min ago`;
  if (ago < 86400) return `${Math.round(ago / 3600)} h ago`;
  return new Date(t).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
}

function stackRunning() {
  return (ctx.state.slow?.launch?.state || "stopped") !== "stopped";
}

function jobRunning() {
  return data?.job?.state === "running";
}

export function init(c) {
  ctx = c;
  $("#sw-fetch").addEventListener("click", fetchRemote);
  $("#sw-build").addEventListener("click", buildDisk);
  $("#sw-restart").addEventListener("click", restartPanel);
  $("#sw-ref").addEventListener("change", (e) => { ref = e.target.value; refresh(); });
}

export function show() {
  refresh();
  timer = setInterval(refresh, 2000);
}

export function hide() {
  clearInterval(timer);
  timer = null;
}

async function refresh() {
  try {
    data = await ctx.api(`software${ref ? `?ref=${encodeURIComponent(ref)}` : ""}`);
  } catch (error) {
    replace($("#sw-commits"), h("p", { class: "hint" }, error.message));
    return;
  }
  if (!ref && data.ref) ref = data.ref;
  render();
}

function render() {
  renderVersions();
  renderRefs();
  renderCommits();
  renderJob();
  $("#sw-build").disabled = jobRunning() || stackRunning() || !data.source;
}

function renderVersions() {
  const run = ctx.state.slow?.launch?.run;
  const rows = [
    ["On disk", data.source, "ros2_ws/src: what the next build compiles",
      data.source && (same(data.source, data.build) ? badge("built", "ok") : badge("not built", "warn"))],
    ["Built", data.build, "install/: what the next launch and panel start run", null],
    ["Panel", data.panel, "this page's server, from the build when it started",
      data.panel && (same(data.panel, data.build) ? badge("current", "ok") : badge("restart it", "warn"))],
    ["Stack", run?.version, run ? "the running launch, from the build when it started" : "not running",
      run?.version && (same(run.version, data.build) ? badge("current", "ok") : badge("restart it", "warn"))],
  ];
  replace($("#sw-versions"),
    h("tr", {}, h("th", {}, ""), h("th", {}, "Commit"), h("th", {}, "Subject"), h("th", {}, "When"), h("th", {}, "")),
    rows.map(([name, v, hint, state]) => h("tr", { title: [hint, v?.via, v?.dirty ? (v.changes || []).join("\n") : null].filter(Boolean).join("\n") },
      h("td", { class: "muted" }, name),
      h("td", { class: "mono" }, label(v)),
      h("td", { class: "sw-subject" },
        v?.dirty ? h("span", { class: "warn-text" }, `uncommitted changes (${(v.changes || []).length}) on `) : null,
        v?.subject || ""),
      h("td", { class: "sw-when" }, when(name === "Built" ? v?.built_at : v?.at)),
      h("td", {}, state))));
}

function renderRefs() {
  const select = $("#sw-ref");
  const refs = data.refs || [];
  const key = refs.map((r) => r.ref).join("|");
  if (select.dataset.key !== key) {
    select.dataset.key = key;
    replace(select, refs.map((r) => h("option", { value: r.ref }, `${r.name} (${r.origin})`)));
  }
  select.value = data.ref || "";
  select.hidden = !refs.length;
  $("#sw-fetch").disabled = !data.git || !data.remote;
  $("#sw-fetch").title = data.remote ? `git fetch ${data.remote}` : "no remote in gui.yaml";
}

function renderCommits() {
  const box = $("#sw-commits");
  if (!data.git) {
    replace(box, h("p", { class: "hint" }, "git is missing in the robot container: add it to deploy/pi/Dockerfile and rebuild the image (deploy/pi/pi setup, then reset)."));
    return;
  }
  if (!data.repo || !(data.refs || []).length) {
    replace(box, h("p", { class: "hint" },
      "Nothing to deploy yet. Fetch from GitHub (the robot needs internet), or send the laptop's commit with ",
      h("code", {}, "deploy/pi/pi push"), "."),
    data.error ? h("p", { class: "hint warn-text" }, data.error) : null);
    return;
  }
  const disk = data.source?.commit;
  const head = data.commits?.[0];
  const blocked = stackRunning() ? "stop the robot stack first" : jobRunning() ? "an update is running" : null;
  const summary = data.behind === 0 ? badge("on disk is the newest", "ok")
    : data.behind > 0 ? badge(`${data.behind} newer than on disk`, "info")
    : disk ? badge(`on disk: ${label(data.source)}, not on this branch`, "warn") : null;
  replace(box,
    h("div", { class: "sw-head" }, summary,
      head ? h("button", { class: "primary small", disabled: !!blocked || (head.commit === disk && !data.source?.dirty),
        title: blocked || "", onclick: () => deploy(head) }, `Deploy newest (${head.short})`) : null,
      blocked ? h("span", { class: "muted small" }, blocked) : null),
    data.error ? h("p", { class: "hint warn-text" }, data.error) : null,
    h("table", { class: "table" },
      (data.commits || []).map((c) => h("tr", { class: c.commit === disk ? "sw-current" : "", title: `${c.subject}\n${c.author}` },
        h("td", { class: "mono" }, c.short),
        h("td", { class: "sw-subject" }, c.subject),
        h("td", { class: "sw-when" }, when(c.time)),
        h("td", { class: "actions" },
          c.commit === disk ? badge(data.source?.dirty ? "on disk, edited" : "on disk", data.source?.dirty ? "warn" : "ok") : null,
          h("button", { class: "small", disabled: !!blocked, title: blocked || `deploy ${c.short}`,
            onclick: () => deploy(c) }, "Deploy"))))));
}

function renderJob() {
  const job = data.job;
  const tail = $("#sw-tail");
  if (!job) {
    $("#sw-job").textContent = "none since the panel started";
    replace(tail);
    return;
  }
  const took = (job.ended || Date.now() / 1000) - job.started;
  const state = job.state === "running" ? job.step : job.state;
  $("#sw-job").textContent = `${job.kind} ${job.target} · ${state} · ${duration(took)}${job.log ? ` · gui_runs/${job.log}` : ""}`;
  const atBottom = tail.scrollTop + tail.clientHeight >= tail.scrollHeight - 20;
  tail.textContent = (data.tail || []).join("\n");
  if (atBottom) tail.scrollTop = tail.scrollHeight;
}

async function deploy(commit) {
  const ok = await ctx.confirm({
    title: `Deploy ${commit.short}?`,
    body: h("div", {},
      h("p", {}, commit.subject),
      h("p", { class: "hint" }, "Writes this commit's ros2_ws/src over what is on disk (uncommitted changes from deploy/pi/pi sync are replaced), builds the packages that changed and restarts the panel. The page reloads by itself."),
    ),
    ok: "Deploy",
  });
  if (!ok) return;
  await ctx.act("software/deploy", { ref: commit.commit }, `deploying ${commit.short}`).catch(() => {});
  refresh();
}

async function fetchRemote() {
  $("#sw-fetch").disabled = true;
  await ctx.act("software/fetch", {}).catch(() => {});
  refresh();
}

async function buildDisk() {
  const ok = await ctx.confirm({
    title: `Build ${label(data.source)}?`,
    body: "colcon builds every package from what is in ros2_ws/src now, then restarts the panel.",
    ok: "Build",
  });
  if (!ok) return;
  await ctx.act("software/build", {}, "building").catch(() => {});
  refresh();
}

async function restartPanel() {
  const ok = await ctx.confirm({
    title: "Restart the panel?",
    body: "The page reconnects in a few seconds. A running launch keeps running.",
    ok: "Restart",
  });
  if (ok) await ctx.act("software/restart_panel", {}).catch(() => {});
}

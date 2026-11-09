// Tune: read and set any node's ROS parameters on the robot, see what changed, export it.

import { $, h, replace, dot, download } from "./util.js";

let ctx;
let selected = null;
let rows = [];
let query = "";
let nodesKey = "";

const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);

function parse(type, text) {
  const t = text.trim();
  if (type === "double") {
    const v = Number(t);
    if (t === "" || !Number.isFinite(v)) throw new Error("not a number");
    return v;
  }
  if (type === "int") {
    const v = Number(t);
    if (!Number.isInteger(v)) throw new Error("not an integer");
    return v;
  }
  if (type.endsWith("[]")) {
    const list = t.startsWith("[") ? JSON.parse(t) : t.split(",").map((s) => s.trim()).filter(Boolean);
    if (!Array.isArray(list)) throw new Error("not a list");
    if (type === "double[]" || type === "int[]") return list.map((s) => {
      const v = Number(s);
      if (!Number.isFinite(v)) throw new Error(`${s} is not a number`);
      return v;
    });
    if (type === "bool[]") return list.map((s) => s === true || s === "true");
    return list.map(String);
  }
  return t;
}

const show = (type, value) => (type.endsWith("[]") ? JSON.stringify(value) : String(value));

async function apply(row, value, input) {
  try {
    await ctx.api("params/set", { node: selected, name: row.name, value });
    row.value = value;
    input?.classList.remove("bad");
    flash(input, "applied");
    render();
  } catch (error) {
    ctx.toast(`${row.name}: ${error.message}`, "error");
    if (input) {
      input.classList.add("bad");
      if (input.type === "checkbox") input.checked = row.value;
    }
  }
}

function flash(input, cls) {
  if (!input) return;
  input.classList.add(cls);
  setTimeout(() => input.classList.remove(cls), 700);
}

function editor(row) {
  if (row.type === "bool") {
    const input = h("input", { type: "checkbox", checked: row.value, disabled: row.read_only });
    input.addEventListener("change", () => apply(row, input.checked, input));
    return input;
  }
  const input = h("input", {
    type: "text", value: show(row.type, row.value), disabled: row.read_only, spellcheck: false,
    class: row.type === "double" || row.type === "int" ? "num" : row.type.endsWith("[]") ? "wide" : "",
  });
  const commit = () => {
    if (input.value === show(row.type, row.value)) return;
    let value;
    try {
      value = parse(row.type, input.value);
    } catch (error) {
      input.classList.add("bad");
      ctx.toast(`${row.name}: ${error.message}`, "error");
      return;
    }
    apply(row, value, input);
  };
  input.addEventListener("keydown", (e) => {
    if (e.key === "Enter") commit();
    if (e.key === "Escape") { input.value = show(row.type, row.value); input.classList.remove("bad"); input.blur(); }
  });
  input.addEventListener("blur", commit);
  return input;
}

function render() {
  const changed = rows.filter((r) => !same(r.value, r.baseline));
  $("#param-changed").textContent = changed.length ? `${changed.length} changed` : "";
  $("#param-export").disabled = !changed.length;
  const visible = rows.filter((r) => !query || r.name.toLowerCase().includes(query));
  const groups = new Map();
  for (const row of visible) {
    const cut = row.name.lastIndexOf(".");
    const group = cut > 0 ? row.name.slice(0, cut) : "";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(row);
  }
  replace($("#params"), visible.length ? [...groups].map(([group, items]) =>
    h("section", { class: "param-group" },
      h("h4", {}, group || "top level"),
      h("table", { class: "param-table" }, h("tbody", {}, items.map((row) => {
        const modified = !same(row.value, row.baseline);
        return h("tr", { class: modified ? "modified" : "", title: row.description || "" },
          h("td", { class: "param-name" }, modified ? h("span", { class: "mod-dot" }) : null,
            row.name.slice(group ? group.length + 1 : 0)),
          h("td", { class: "param-value" }, editor(row),
            row.range ? h("span", { class: "muted small" }, ` ${row.range[0]} … ${row.range[1]}`) : null),
          h("td", { class: "param-type muted" }, row.read_only ? "read-only" : row.type),
          h("td", { class: "param-was muted" }, modified ? [`was ${show(row.type, row.baseline)} `,
            h("button", { class: "link", onclick: () => apply(row, row.baseline, null) }, "revert")] : null));
      }))))) : h("p", { class: "empty" }, selected ? "No parameter matches." : "Pick a node."));
}

async function load(node) {
  selected = node;
  $("#param-node").textContent = node;
  document.querySelectorAll("#nodes li").forEach((li) => li.classList.toggle("active", li.dataset.node === node));
  const controller = (ctx.state.slow?.ros?.controllers || []).find((c) => `/${c.name}` === node);
  $("#param-note").textContent = controller?.type?.includes("RlPolicy")
    ? "RL policy settings come from the exported policy and are read-only."
    : controller ? "Applied live: the controller validates each value and refuses an invalid set." : "Nodes that read parameters only at start ignore changes until restarted.";
  replace($("#params"), h("p", { class: "empty" }, "Loading…"));
  try {
    rows = await ctx.api(`params?node=${encodeURIComponent(node)}`);
    render();
  } catch (error) {
    rows = [];
    replace($("#params"), h("p", { class: "empty warn-text" }, error.message));
  }
}

function renderNodes(slow) {
  const ros = slow.ros || {};
  const controllers = Object.fromEntries((ros.controllers || []).map((c) => [`/${c.name}`, c]));
  const nodes = [...(ros.nodes || [])];
  for (const name of Object.keys(controllers)) if (!nodes.includes(name)) nodes.push(name);
  nodes.sort((a, b) => {
    const ca = controllers[a], cb = controllers[b];
    if (!!ca !== !!cb) return ca ? -1 : 1;
    if (ca && cb && (ca.state === "active") !== (cb.state === "active")) return ca.state === "active" ? -1 : 1;
    return a.localeCompare(b);
  });
  const key = JSON.stringify(nodes.map((n) => [n, controllers[n]?.state]));
  if (key === nodesKey) return;
  nodesKey = key;
  replace($("#nodes"), nodes.length ? nodes.map((node) => {
    const c = controllers[node];
    return h("li", { "data-node": node, class: node === selected ? "active" : "", onclick: () => load(node) },
      dot(c ? (c.state === "active" ? "ok" : "off") : "none"), h("span", { class: "mono" }, node.replace(/^\//, "")),
      c ? h("span", { class: "muted small" }, c.state) : null);
  }) : h("li", { class: "empty" }, "No ROS nodes: start the stack."));
}

export function init(context) {
  ctx = context;
  $("#param-filter").addEventListener("input", (e) => { query = e.target.value.toLowerCase(); render(); });
  $("#param-reload").addEventListener("click", () => selected && load(selected));
  $("#param-export").addEventListener("click", async () => {
    const response = await fetch("/api/params/export");
    download("tuned.yaml", await response.text(), "text/yaml");
  });
  ctx.on("slow", renderNodes);
  render();
}

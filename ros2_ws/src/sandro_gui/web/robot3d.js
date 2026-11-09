// The robot in 3D: CAD meshes posed from the joint angles, tilted by the IMU.
//
// Coordinates are the IMU frame of the MJCF (x forward, y left, z up), the
// body frame of the controllers. Each leg is drawn with the controllers'
// five-bar kinematics: a crank turns about its motor pivot, a lower link moves
// with its knee and turns by its link angle, the wheel follows the axle and
// spins by its joint angle. Yaw is removed from the IMU attitude so the view
// does not drift with the gyro.

import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";
import { fiveBar, LEG } from "./util.js";

const RGB = [0xe5534b, 0x57ab5a, 0x539bf5];
const STAND = { right_motor_inner: 0.411, right_motor_outer: 0.217, right_wheel: 0,
  left_motor_inner: 0.215, left_motor_outer: 0.413, left_wheel: 0 };
// Used when the server has no model: robot_full_floating.xml, 2026-11 body.
const FALLBACK_LEGS = {
  right: { direction: 1, inner_pivot: [-0.0131, -0.101, -0.0922], outer_pivot: [0.0769, -0.101, -0.0922],
    theta_cad: [0.3166, 0.6359], knee_y: { inner: -0.1045, outer: -0.1225 }, wheel_y: -0.1482, wheel_on: "outer" },
  left: { direction: -1, inner_pivot: [0.0771, 0.101, -0.0922], outer_pivot: [-0.0129, 0.101, -0.0922],
    theta_cad: [0.8179, 1.1031], knee_y: { inner: 0.1122, outer: 0.1122 }, wheel_y: 0.1489, wheel_on: "inner" },
};

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

function signedAngleY(a, b) {
  return Math.atan2(a.z * b.x - a.x * b.z, a.x * b.x + a.z * b.z);
}

function aboutY(now, angle, cad) {
  return new THREE.Matrix4().makeTranslation(now.x, now.y, now.z)
    .multiply(new THREE.Matrix4().makeRotationY(angle))
    .multiply(new THREE.Matrix4().makeTranslation(-cad.x, -cad.y, -cad.z));
}

function triad(length, radius, opacity = 1) {
  const group = new THREE.Group();
  const shaft = new THREE.CylinderGeometry(radius, radius, length, 10);
  const tip = new THREE.ConeGeometry(radius * 2.4, length * 0.22, 14);
  for (let axis = 0; axis < 3; axis++) {
    const material = new THREE.MeshBasicMaterial({ color: RGB[axis], depthTest: false, transparent: true, opacity });
    const arm = new THREE.Group();
    const s = new THREE.Mesh(shaft, material);
    s.position.y = length / 2;
    const t = new THREE.Mesh(tip, material);
    t.position.y = length + length * 0.11;
    arm.add(s, t);
    if (axis === 0) arm.rotation.z = -Math.PI / 2;
    if (axis === 2) arm.rotation.x = Math.PI / 2;
    for (const m of [s, t]) m.renderOrder = 10;
    group.add(arm);
  }
  return group;
}

export class RobotView {
  constructor(container) {
    this.container = container;
    this.legs = FALLBACK_LEGS;
    this.geometry = LEG;
    this.wheelRadius = 0.0742;
    this.options = { skeleton: false, frames: true, attitude: true };
    this.groups = {};
    this.labels = [];
    this.dirty = true;
    this.visible = false;

    const renderer = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.append(renderer.domElement);
    this.overlay = document.createElement("div");
    this.overlay.className = "labels3d";
    container.append(this.overlay);

    const scene = (this.scene = new THREE.Scene());
    scene.background = new THREE.Color(0x15181b);
    const camera = (this.camera = new THREE.PerspectiveCamera(32, 1, 0.01, 20));
    camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(camera, renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.addEventListener("change", () => { this.dirty = true; });
    this.resetView();

    scene.add(new THREE.HemisphereLight(0xe8edf2, 0x30343a, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.9);
    sun.position.set(0.8, -0.5, 1.4);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    Object.assign(sun.shadow.camera, { left: -0.4, right: 0.4, top: 0.4, bottom: -0.4, near: 0.1, far: 4 });
    sun.shadow.bias = -0.0005;
    scene.add(sun);
    const fill = new THREE.DirectionalLight(0xb8c4d0, 0.6);
    fill.position.set(-0.7, 0.8, 0.5);
    scene.add(fill);

    const grid = new THREE.GridHelper(2, 20, 0x3a4048, 0x262a30);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), new THREE.ShadowMaterial({ opacity: 0.35 }));
    floor.receiveShadow = true;
    floor.position.z = -0.0005;
    scene.add(floor);
    const world = triad(0.08, 0.0018, 0.55);
    scene.add(world);
    this.label("world", world, new THREE.Vector3(0.0, -0.02, 0.0), "dim");

    this.robot = new THREE.Group();
    scene.add(this.robot);
    this.body = new THREE.Group();
    this.robot.add(this.body);
    this.torso = new THREE.Group();
    this.body.add(this.torso);

    this.frames = new THREE.Group();
    this.body.add(this.frames);
    this.imuFrame = triad(0.07, 0.002);
    this.frames.add(this.imuFrame);
    this.label("imu", this.imuFrame, new THREE.Vector3(0.088, 0, 0.004));
    // The chip's own axes: x is body left, so its -x tip points out of the right side.
    this.chipFrame = triad(0.035, 0.0012, 0.6);
    this.frames.add(this.chipFrame);
    this.label("BNO085 chip", this.chipFrame, new THREE.Vector3(-0.012, -0.05, 0.0), "dim");
    this.baseFrame = triad(0.05, 0.0015, 0.85);
    this.frames.add(this.baseFrame);
    this.label("base_link", this.baseFrame, new THREE.Vector3(0.066, 0, -0.004), "dim");

    this.skeleton = new THREE.Group();
    this.body.add(this.skeleton);
    this.buildSkeleton();
    this.placeFrames([1, 0, 0, 0]);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.pose = { ...STAND };
    this.applyPose();
    const tick = () => {
      requestAnimationFrame(tick);
      if (!this.visible) return;
      const moved = this.controls.update();
      if (moved || this.dirty) {
        this.dirty = false;
        this.renderer.render(this.scene, this.camera);
        this.placeLabels();
      }
    };
    tick();
  }

  resetView() {
    this.camera.position.set(0.62, -0.72, 0.38);
    this.controls.target.set(0.02, 0, 0.14);
    this.controls.update();
    this.dirty = true;
  }

  resize() {
    const { clientWidth: w, clientHeight: hgt } = this.container;
    if (!w || !hgt) return;
    this.renderer.setSize(w, hgt);
    this.camera.aspect = w / hgt;
    this.camera.updateProjectionMatrix();
    this.dirty = true;
  }

  setVisible(visible) {
    this.visible = visible;
    if (visible) {
      this.resize();
      this.dirty = true;
    }
  }

  setOptions(options) {
    Object.assign(this.options, options);
    this.skeleton.visible = this.options.skeleton;
    this.frames.visible = this.options.frames;
    for (const label of this.labels) label.hidden = !this.options.frames;
    this.applyPose();
  }

  label(text, object, offset, kind = "") {
    const el = document.createElement("span");
    el.className = `label3d ${kind}`;
    el.textContent = text;
    this.overlay.append(el);
    this.labels.push({ el, object, offset, hidden: false });
  }

  placeLabels() {
    const { clientWidth: w, clientHeight: hgt } = this.container;
    const p = new THREE.Vector3();
    for (const label of this.labels) {
      label.object.updateWorldMatrix(true, false);
      p.copy(label.offset).applyMatrix4(label.object.matrixWorld).project(this.camera);
      const off = label.hidden || p.z > 1;
      label.el.style.display = off ? "none" : "";
      if (!off) label.el.style.transform = `translate(${((p.x + 1) / 2) * w}px, ${((1 - p.y) / 2) * hgt}px)`;
    }
  }

  placeFrames(chipQuat) {
    // base_link of sandro.urdf.xacro: 45 mm above the centre of the four motor pivots.
    const r = this.legs.right, l = this.legs.left;
    const centre = v3(r.inner_pivot).add(v3(r.outer_pivot)).add(v3(l.inner_pivot)).add(v3(l.outer_pivot)).multiplyScalar(0.25);
    this.baseFrame.position.copy(centre).add(new THREE.Vector3(0, 0, 0.045));
    const [w, x, y, z] = chipQuat;
    this.chipFrame.quaternion.set(x, y, z, w);
  }

  buildSkeleton() {
    this.skeleton.clear();
    const material = new THREE.LineBasicMaterial({ color: 0xf2f4f7, depthTest: false, transparent: true, opacity: 0.9 });
    const joint = new THREE.MeshBasicMaterial({ color: 0xf2f4f7, depthTest: false });
    const ball = new THREE.SphereGeometry(0.004, 12, 8);
    this.bones = {};
    for (const side of ["right", "left"]) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(8 * 3), 3));
      const lines = new THREE.LineSegments(geometry, material);
      lines.renderOrder = 11;
      const balls = [];
      for (let k = 0; k < 5; k++) {
        const m = new THREE.Mesh(ball, joint);
        m.renderOrder = 11;
        balls.push(m);
        this.skeleton.add(m);
      }
      this.skeleton.add(lines);
      this.bones[side] = { lines, balls };
    }
    this.skeleton.visible = this.options.skeleton;
  }

  async load() {
    let meta = null;
    for (let attempt = 0; attempt < 60; attempt++) {
      const response = await fetch("/api/model");
      if (response.status === 200) {
        meta = await response.json();
        break;
      }
      if (response.status === 503) {
        const data = await response.json().catch(() => ({}));
        throw new Error(data.error || "no 3D model on the server");
      }
      await new Promise((r) => setTimeout(r, 1500));
    }
    if (!meta) throw new Error("3D model not ready");
    const buffer = await (await fetch("/api/model.bin")).arrayBuffer();
    this.legs = meta.legs;
    this.geometry = meta.geometry;
    this.wheelRadius = meta.wheel_radius;
    const geometries = meta.meshes.map((m) => {
      const q = new Uint16Array(buffer, m.vertex_offset, m.vertices * 3);
      const positions = new Float32Array(m.vertices * 3);
      for (let i = 0; i < m.vertices; i++) {
        for (let k = 0; k < 3; k++) positions[3 * i + k] = m.origin[k] + q[3 * i + k] * m.scale[k];
      }
      const Index = m.index === "u2" ? Uint16Array : Uint32Array;
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      geometry.setIndex(new THREE.BufferAttribute(new Index(buffer, m.index_offset, m.faces * 3), 1));
      geometry.computeVertexNormals();
      return geometry;
    });
    const materials = {};
    for (const [name, info] of Object.entries(meta.groups)) {
      if (name === "torso") continue;
      const group = new THREE.Group();
      group.matrixAutoUpdate = false;
      group.userData = info;
      this.body.add(group);
      this.groups[name] = group;
    }
    for (const part of meta.parts) {
      const key = part.color.join(",");
      materials[key] ||= new THREE.MeshStandardMaterial({
        color: new THREE.Color(...part.color), roughness: 0.72, metalness: 0.05, flatShading: true,
      });
      const mesh = new THREE.Mesh(geometries[part.mesh], materials[key]);
      mesh.matrixAutoUpdate = false;
      mesh.matrix.fromArray(part.matrix);
      mesh.castShadow = true;
      (this.groups[part.group] || this.torso).add(mesh);
    }
    this.torso.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.torso);
    this.torsoCorners = [];
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
      this.torsoCorners.push(new THREE.Vector3(x, y, z));
    }
    this.placeFrames(meta.imu_chip_quat);
    this.loaded = true;
    this.applyPose();
    return meta;
  }

  // joints: {name: radians}; quaternion: [x, y, z, w] body in world, or null.
  update(joints, quaternion) {
    this.pose = { ...STAND, ...joints };
    this.quaternion = quaternion;
    this.applyPose();
  }

  legPoints(side) {
    const leg = this.legs[side];
    const d = leg.direction;
    const inner = v3(leg.inner_pivot);
    const toBody = (p, y) => new THREE.Vector3(inner.x + d * p.x, y, inner.z - p.y);
    const solve = (ti, to) => {
      const fk = fiveBar(ti, to, this.geometry);
      if (!fk) return null;
      return {
        kneeInner: toBody(fk.kneeInner, leg.knee_y.inner),
        kneeOuter: toBody(fk.kneeOuter, leg.knee_y.outer),
        axle: toBody(fk, leg.wheel_y),
      };
    };
    return {
      leg, inner, outer: v3(leg.outer_pivot),
      cad: solve(leg.theta_cad[0], leg.theta_cad[1]),
      now: solve(this.pose[`${side}_motor_inner`], this.pose[`${side}_motor_outer`]),
    };
  }

  applyPose() {
    const axles = [];
    for (const side of ["right", "left"]) {
      const { leg, inner, outer, cad, now } = this.legPoints(side);
      if (!now || !cad) continue;
      axles.push(now.axle);
      const flat = (v) => new THREE.Vector3(v.x, 0, v.z);
      const crank = {
        inner: signedAngleY(flat(cad.kneeInner.clone().sub(inner)), flat(now.kneeInner.clone().sub(inner))),
        outer: signedAngleY(flat(cad.kneeOuter.clone().sub(outer)), flat(now.kneeOuter.clone().sub(outer))),
      };
      const lower = {
        inner: signedAngleY(flat(cad.axle.clone().sub(cad.kneeInner)), flat(now.axle.clone().sub(now.kneeInner))),
        outer: signedAngleY(flat(cad.axle.clone().sub(cad.kneeOuter)), flat(now.axle.clone().sub(now.kneeOuter))),
      };
      for (const group of Object.values(this.groups)) {
        const info = group.userData;
        if (info.side !== side) continue;
        let matrix;
        if (info.kind === "crank") {
          const pivot = info.crank === "inner" ? inner : outer;
          matrix = aboutY(pivot, crank[info.crank], pivot);
        } else if (info.kind === "lower") {
          const knee = info.crank === "inner" ? "kneeInner" : "kneeOuter";
          matrix = aboutY(now[knee], lower[info.crank], cad[knee]);
        } else if (info.kind === "wheel") {
          matrix = aboutY(now.axle, lower[info.carrier] + (this.pose[`${side}_wheel`] || 0), cad.axle);
        }
        if (matrix) {
          group.matrix.copy(matrix);
          group.matrixWorldNeedsUpdate = true;
        }
      }
      const bone = this.bones[side];
      const segments = [[inner, now.kneeInner], [outer, now.kneeOuter], [now.kneeInner, now.axle], [now.kneeOuter, now.axle]];
      const array = bone.lines.geometry.attributes.position.array;
      segments.flat().forEach((p, i) => array.set([p.x, p.y, p.z], 3 * i));
      bone.lines.geometry.attributes.position.needsUpdate = true;
      [inner, outer, now.kneeInner, now.kneeOuter, now.axle].forEach((p, i) => bone.balls[i].position.copy(p));
    }

    const attitude = new THREE.Quaternion();
    if (this.options.attitude && this.quaternion) {
      const [x, y, z, w] = this.quaternion;
      const q = new THREE.Quaternion(x, y, z, w).normalize();
      const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      attitude.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -yaw).multiply(q);
    }
    this.robot.quaternion.copy(attitude);
    const lows = [];
    const centre = new THREE.Vector3();
    for (const axle of axles) {
      const p = axle.clone().applyQuaternion(attitude);
      lows.push(p.z - this.wheelRadius);
      centre.add(p);
    }
    for (const corner of this.torsoCorners || []) lows.push(corner.clone().applyQuaternion(attitude).z);
    if (axles.length) centre.multiplyScalar(1 / axles.length);
    this.robot.position.set(-centre.x, -centre.y, lows.length ? -Math.min(...lows) : 0.27);
    this.dirty = true;
  }

  // Leg height (pivot line to axle) and fore-aft axle position, per side, and the
  // body roll (right side down positive) those heights give with both wheels on level ground.
  legState() {
    const out = {};
    for (const side of ["right", "left"]) {
      const { inner, outer, now } = this.legPoints(side);
      if (!now) continue;
      const mid = inner.clone().add(outer).multiplyScalar(0.5);
      out[side] = { height: mid.z - now.axle.z, forward: now.axle.x - mid.x };
    }
    if (out.right && out.left) {
      const track = Math.abs(this.legs.left.wheel_y - this.legs.right.wheel_y);
      out.roll = Math.atan2(out.left.height - out.right.height, track);
    }
    return out;
  }
}

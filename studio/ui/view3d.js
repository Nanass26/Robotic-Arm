// ORION Studio — vue 3D (Three.js). Repère monde Z vers le haut, unités en mètres.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { buildProceduralRobot, makeGhost, ghostMaterial, applyFrames, makeMaterials } from './robot3d.js';
import { cssVar } from './dom.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const OBJ_COLORS = [0x2a78d6, 0xeb6834, 0x1baf7a, 0xeda100, 0xe87ba4];

export class View3D {
  constructor(container, app) {
    this.app = app;
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    container.prepend(this.renderer.domElement);
    this.renderer.domElement.setAttribute('aria-label', 'Vue 3D du robot');
    this.renderer.domElement.tabIndex = 0;

    this.scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0.95, -0.85, 0.72);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.12, 0, 0.24);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.maxDistance = 8;
    this.controls.minDistance = 0.15;

    // Lumières
    const hemi = new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.55);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xffffff, 1.8);
    sun.position.set(0.8, -0.6, 1.6);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -1; sun.shadow.camera.right = 1;
    sun.shadow.camera.top = 1; sun.shadow.camera.bottom = -1;
    sun.shadow.camera.near = 0.1; sun.shadow.camera.far = 5;
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.02;
    sun.shadow.radius = 3;
    this.scene.add(sun);
    this.sun = sun;

    // Plan de travail : ombres + grille
    this.ground = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.ShadowMaterial({ opacity: 0.18 }));
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);
    this.grid = new THREE.Group();
    this.scene.add(this.grid);
    this.axes = new THREE.AxesHelper(0.12);
    this.axes.position.z = 0.0005;
    this.scene.add(this.axes);

    // Groupes dynamiques
    this.robotLayer = new THREE.Group();
    this.scene.add(this.robotLayer);
    this.ghostLayer = new THREE.Group();
    this.ghostLayer.visible = false;
    this.scene.add(this.ghostLayer);
    this.overlay = new THREE.Group();
    this.scene.add(this.overlay);
    this.objectsLayer = new THREE.Group();
    this.scene.add(this.objectsLayer);

    // Trace du TCP (tampon circulaire) et trace de consigne
    this.trailMax = 3000;
    this.trail = this.makeTrail(0x0b8c87);
    this.trailPts = [];
    this.scene.add(this.trail);

    // Cible TCP manipulable
    this.target = new THREE.Object3D();
    this.scene.add(this.target);
    this.targetHelper = new THREE.AxesHelper(0.06);
    this.target.add(this.targetHelper);
    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.setSize(0.75);
    this.gizmo.setSpace('world');
    this.gizmo.attach(this.target);
    this.gizmoHelper = this.gizmo.getHelper();
    this.scene.add(this.gizmoHelper);
    this.gizmoEnabled = true;
    this.gizmo.addEventListener('dragging-changed', (e) => {
      this.controls.enabled = !e.value;
      if (e.value) this.app.onTargetDragStart?.();
      else this.app.onTargetDragEnd?.(this.targetMatrixRowMajor());
    });
    this.gizmo.addEventListener('objectChange', () => {
      if (this.gizmo.dragging) this.app.onTargetDrag?.(this.targetMatrixRowMajor());
    });

    this.frameHelpers = new THREE.Group();
    this.frameHelpers.visible = false;
    this.scene.add(this.frameHelpers);
    this.capsuleLayer = new THREE.Group();
    this.capsuleLayer.visible = false;
    this.scene.add(this.capsuleLayer);
    this.workspace = null;

    // Poussée à la souris (Maj + glisser)
    this.raycaster = new THREE.Raycaster();
    this.pushLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ color: 0xc8102e }));
    this.pushLine.visible = false;
    this.scene.add(this.pushLine);
    this.setupPointer();

    this.options = { shadows: true, trail: true, frames: false, capsules: false, ghost: true, gizmo: true };
    this.applyTheme();
    this.resize();
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(container);
  }

  makeTrail(color) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(this.trailMax * 3), 3));
    g.setDrawRange(0, 0);
    const l = new THREE.Line(g, new THREE.LineBasicMaterial({ color }));
    l.frustumCulled = false;
    return l;
  }

  applyTheme() {
    const dark = getComputedStyle(document.documentElement).colorScheme.includes('dark');
    this.dark = dark;
    const major = new THREE.Color(cssVar('--grid-major') || '#b5c0c8');
    const minor = new THREE.Color(cssVar('--grid-minor') || '#d3dbe0');
    const accent = new THREE.Color(cssVar('--accent') || '#0b8c87');
    this.grid.clear();
    const g1 = new THREE.GridHelper(2, 40, minor, minor);
    g1.rotation.x = Math.PI / 2;
    const g2 = new THREE.GridHelper(2, 8, major, major);
    g2.rotation.x = Math.PI / 2;
    g2.position.z = 0.0002;
    this.grid.add(g1, g2);
    this.ground.material.opacity = dark ? 0.35 : 0.18;
    this.trail.material.color = accent;
    if (this.mats) {
      this.mats.accent.color = accent;
      this.mats.shell.color = new THREE.Color(dark ? 0xdfe5ea : 0xf1f4f6);
    }
  }

  resize() {
    const r = this.container.getBoundingClientRect();
    const w = Math.max(10, r.width), hgt = Math.max(10, r.height);
    this.renderer.setSize(w, hgt, false);
    this.camera.aspect = w / hgt;
    this.camera.updateProjectionMatrix();
  }

  // ---------------------------------------------------------------- robot
  buildRobot(kin, params, cad = null) {
    this.robotLayer.clear();
    this.ghostLayer.clear();
    this.mats = this.mats || makeMaterials(this.dark);
    this.robot = buildProceduralRobot(kin, params, this.mats);
    if (cad) this.useCad(cad);
    this.robotLayer.add(this.robot.root);
    this.ghost = makeGhost(this.robot, ghostMaterial(new THREE.Color(cssVar('--accent') || '#0b8c87')));
    this.ghostLayer.add(this.ghost.root);
    this.buildFrameHelpers(kin);
    this.setShadows(this.options.shadows);
    this.kin = kin;
    this.params = params;
  }

  /** Remplace la géométrie procédurale par les maillages CAO (pièces imprimables). */
  useCad(cad) {
    if (!cad || !this.robot) return;
    const keepGripper = this.robot.gripper.group;
    this.robot.links.forEach((g, i) => {
      for (const c of [...g.children]) if (c !== keepGripper) g.remove(c);
      for (const part of cad.links[i] || []) {
        const m = new THREE.Mesh(part.geometry, part.material || this.mats.shell);
        m.userData.link = i;
        m.userData.part = part.id;
        m.castShadow = true; m.receiveShadow = true;
        g.add(m);
      }
    });
    this.robot.meshes = [];
    this.robot.root.traverse((o) => { if (o.isMesh) this.robot.meshes.push(o); });
    this.cadActive = true;
  }

  buildFrameHelpers(kin) {
    this.frameHelpers.clear();
    this.frameAxes = [];
    for (let i = 0; i <= kin.n + 1; i++) {
      const a = new THREE.AxesHelper(i === kin.n + 1 ? 0.07 : 0.05);
      a.matrixAutoUpdate = false;
      this.frameHelpers.add(a);
      this.frameAxes.push(a);
    }
  }

  setShadows(on) {
    this.options.shadows = on;
    this.renderer.shadowMap.enabled = on;
    this.ground.visible = on;
    this.robotLayer.traverse((o) => { if (o.isMesh) o.castShadow = on; });
  }

  /** Met à jour la pose du robot depuis un résultat de FK. */
  updateRobot(fk, gripperOpening) {
    if (!this.robot) return;
    applyFrames(this.robot.links, fk.frames);
    this.robot.gripper.update?.(gripperOpening);
    if (this.frameHelpers.visible) {
      const all = [...fk.frames, fk.tcp];
      all.forEach((F, i) => {
        const a = this.frameAxes[i];
        if (!a) return;
        a.matrix.set(F[0], F[1], F[2], F[3], F[4], F[5], F[6], F[7], F[8], F[9], F[10], F[11], 0, 0, 0, 1);
      });
    }
  }

  updateGhost(frames, visible = true) {
    if (!this.ghost) return;
    this.ghostLayer.visible = visible && this.options.ghost;
    if (visible) applyFrames(this.ghost.links, frames);
  }

  setGhostColor(ok) {
    const c = ok ? new THREE.Color(cssVar('--accent') || '#0b8c87') : new THREE.Color(0xd03b3b);
    this.ghost?.root.traverse((o) => { if (o.isMesh) o.material.color = c; });
  }

  // ---------------------------------------------------------------- cible (gizmo)
  targetMatrixRowMajor() {
    this.target.updateMatrixWorld();
    const e = this.target.matrixWorld.elements; // colonne-majeur
    return [e[0], e[4], e[8], e[12], e[1], e[5], e[9], e[13], e[2], e[6], e[10], e[14], 0, 0, 0, 1];
  }

  setTargetFromRowMajor(T) {
    const m = new THREE.Matrix4().set(T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9], T[10], T[11], 0, 0, 0, 1);
    m.decompose(this.target.position, this.target.quaternion, this.target.scale);
  }

  setGizmoMode(mode) { this.gizmo.setMode(mode); }
  setGizmoSpace(space) { this.gizmo.setSpace(space); }
  setGizmoVisible(v) {
    this.options.gizmo = v;
    this.gizmoHelper.visible = v;
    this.gizmo.enabled = v;
    this.targetHelper.visible = v;
  }

  // ---------------------------------------------------------------- trace TCP
  pushTrail(p) {
    if (!this.options.trail) return;
    const last = this.trailPts[this.trailPts.length - 1];
    if (last && Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) < 0.0015) return;
    this.trailPts.push(p);
    if (this.trailPts.length > this.trailMax) this.trailPts.shift();
    const arr = this.trail.geometry.attributes.position.array;
    this.trailPts.forEach((q, i) => { arr[i * 3] = q[0]; arr[i * 3 + 1] = q[1]; arr[i * 3 + 2] = q[2]; });
    this.trail.geometry.attributes.position.needsUpdate = true;
    this.trail.geometry.setDrawRange(0, this.trailPts.length);
  }
  clearTrail() { this.trailPts = []; this.trail.geometry.setDrawRange(0, 0); }
  setTrailVisible(v) { this.options.trail = v; this.trail.visible = v; if (!v) this.clearTrail(); }

  // ---------------------------------------------------------------- espace de travail
  showWorkspace(sample) {
    if (this.workspace) { this.scene.remove(this.workspace); this.workspace.geometry.dispose(); this.workspace = null; }
    if (!sample) return;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(sample.points, 3));
    const col = new Float32Array(sample.manipulability.length * 3);
    // Rampe séquentielle bleue (clair → foncé = manipulabilité faible → forte)
    const lo = new THREE.Color('#cde2fb'), hi = new THREE.Color('#104281'), c = new THREE.Color();
    sample.manipulability.forEach((m, i) => { c.copy(lo).lerp(hi, Math.sqrt(m)); col[i * 3] = c.r; col[i * 3 + 1] = c.g; col[i * 3 + 2] = c.b; });
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    this.workspace = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.006, vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false }));
    this.scene.add(this.workspace);
  }

  // ---------------------------------------------------------------- objets manipulables
  syncObjects(objects) {
    const byId = new Map(this.objectsLayer.children.map((m) => [m.userData.id, m]));
    objects.forEach((o, k) => {
      let m = byId.get(o.id);
      if (!m) {
        m = new THREE.Mesh(new THREE.BoxGeometry(o.size, o.size, o.size), new THREE.MeshStandardMaterial({ color: o.color ?? OBJ_COLORS[k % OBJ_COLORS.length], roughness: 0.5, metalness: 0.05 }));
        m.castShadow = true; m.receiveShadow = true;
        m.userData.id = o.id;
        m.matrixAutoUpdate = false;
        this.objectsLayer.add(m);
      }
      const T = o.pose;
      m.matrix.set(T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9], T[10], T[11], 0, 0, 0, 1);
      m.matrixWorldNeedsUpdate = true;
      byId.delete(o.id);
    });
    for (const m of byId.values()) this.objectsLayer.remove(m);
  }

  // ---------------------------------------------------------------- capsules de collision
  showCapsules(caps, collidingLinks = new Set()) {
    this.capsuleLayer.clear();
    if (!this.capsuleLayer.visible) return;
    for (const c of caps) {
      const a = new THREE.Vector3(...c.w0), b = new THREE.Vector3(...c.w1);
      const len = a.distanceTo(b);
      const m = new THREE.Mesh(new THREE.CapsuleGeometry(c.r, Math.max(len, 1e-4), 4, 12),
        new THREE.MeshBasicMaterial({ color: collidingLinks.has(c.link) ? 0xd03b3b : 0x2a78d6, wireframe: true, transparent: true, opacity: 0.45 }));
      m.position.copy(a).add(b).multiplyScalar(0.5);
      if (len > 1e-6) m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
      this.capsuleLayer.add(m);
    }
  }

  // ---------------------------------------------------------------- caméra
  setView(name) {
    const t = this.controls.target;
    const d = this.camera.position.distanceTo(t);
    const dirs = { iso: [1, -0.9, 0.75], front: [1, 0, 0.12], side: [0, -1, 0.12], top: [0.0001, -0.0001, 1], back: [-1, 0.9, 0.75] };
    const v = new THREE.Vector3(...(dirs[name] || dirs.iso)).normalize().multiplyScalar(d);
    this.camera.position.copy(t).add(v);
    this.camera.lookAt(t);
    this.controls.update();
  }

  // ---------------------------------------------------------------- pointeur (poussée)
  setupPointer() {
    const el = this.renderer.domElement;
    let drag = null;
    const ndc = (e) => {
      const r = el.getBoundingClientRect();
      return new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    };
    el.addEventListener('pointerdown', (e) => {
      if (!e.shiftKey || !this.robot) return;
      this.raycaster.setFromCamera(ndc(e), this.camera);
      const hits = this.raycaster.intersectObjects(this.robot.meshes, false);
      const hit = hits.find((hh) => hh.object.userData.link > 0);
      if (!hit) return;
      const link = hit.object.userData.link;
      const group = this.robot.links[link];
      group.updateMatrixWorld();
      const local = group.worldToLocal(hit.point.clone());
      const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(this.camera.getWorldDirection(new THREE.Vector3()).negate(), hit.point);
      drag = { link, local, plane, pointerId: e.pointerId };
      el.setPointerCapture(e.pointerId);
      this.controls.enabled = false;
      e.preventDefault();
    });
    el.addEventListener('pointermove', (e) => {
      if (!drag) return;
      this.raycaster.setFromCamera(ndc(e), this.camera);
      const p = new THREE.Vector3();
      if (!this.raycaster.ray.intersectPlane(drag.plane, p)) return;
      const group = this.robot.links[drag.link];
      group.updateMatrixWorld();
      const w = drag.local.clone().applyMatrix4(group.matrixWorld);
      const f = p.clone().sub(w).multiplyScalar(260);
      if (f.length() > 60) f.setLength(60);
      this.app.onPush?.({ link: drag.link, local: [drag.local.x, drag.local.y, drag.local.z], force: [f.x, f.y, f.z] });
      this.pushLine.geometry.setFromPoints([w, p]);
      this.pushLine.visible = true;
    });
    const end = () => {
      if (!drag) return;
      drag = null;
      this.controls.enabled = true;
      this.pushLine.visible = false;
      this.app.onPush?.(null);
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }

  render() {
    this.controls.update();
    // Mise à jour de la ligne de poussée pendant le mouvement
    this.renderer.render(this.scene, this.camera);
  }
}

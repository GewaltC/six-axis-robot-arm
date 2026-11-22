import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

export class RobotStudioScene {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1018);
    this.scene.fog = new THREE.FogExp2(0x0b1018, 0.14);

    const w = container.clientWidth || window.innerWidth;
    const h = container.clientHeight || window.innerHeight;

    this.camera = new THREE.PerspectiveCamera(42, w / h, 0.02, 50);
    this.camera.position.set(1.25, 1.05, 1.45);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(this.renderer.domElement);

    // PBR Environment map for realistic brushed aluminum & anodized metal reflections
    const pmremGenerator = new THREE.PMREMGenerator(this.renderer);
    this.envRT = pmremGenerator.fromScene(new RoomEnvironment(), 0.04);
    this.scene.environment = this.envRT.texture;
    this.scene.environmentIntensity = 0.72;
    pmremGenerator.dispose();

    // OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 8.0;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.target.set(0, 0.42, 0);
    this.controls.update();

    this._setupLights();
    this._setupStage();
    this._setupIkGizmo();
    this._setupTrail();
    this._setupWorkspaceEnvelope();

    // Camera transition state
    this.camAnim = null;

    window.addEventListener('resize', () => this.resize());
  }

  _setupLights() {
    const hemi = new THREE.HemisphereLight(0xdce8ff, 0x1a2230, 0.65);
    this.scene.add(hemi);

    // Key light (warm studio daylight)
    const key = new THREE.DirectionalLight(0xfff6eb, 2.1);
    key.position.set(2.4, 4.2, 2.6);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.5;
    key.shadow.camera.far = 12;
    const d = 1.6;
    key.shadow.camera.left = -d;
    key.shadow.camera.right = d;
    key.shadow.camera.top = d;
    key.shadow.camera.bottom = -d;
    key.shadow.bias = -0.0003;
    key.shadow.normalBias = 0.015;
    this.scene.add(key);

    // Cool technical rim light
    const rim = new THREE.DirectionalLight(0x6ecbff, 1.05);
    rim.position.set(-2.8, 2.2, -2.0);
    this.scene.add(rim);

    // Subtle front-left fill
    const fill = new THREE.DirectionalLight(0xc2d8ff, 0.55);
    fill.position.set(-1.5, 1.4, 2.4);
    this.scene.add(fill);
  }

  _setupStage() {
    this.stageGroup = new THREE.Group();
    this.scene.add(this.stageGroup);

    // Shadow-receiving matte studio floor
    const floorGeo = new THREE.CircleGeometry(6.5, 96);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0f1622,
      roughness: 0.85,
      metalness: 0.12,
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.002;
    floor.receiveShadow = true;
    this.floorMesh = floor;
    this.stageGroup.add(floor);

    // Precision CAD grid
    this.gridMajor = new THREE.GridHelper(6, 30, 0x26415e, 0x172436);
    this.gridMajor.position.y = 0.0005;
    this.gridMajor.material.opacity = 0.7;
    this.gridMajor.material.transparent = true;
    this.stageGroup.add(this.gridMajor);

    // Industrial mounting flange / pedestal plate right under robot base
    const plateGroup = new THREE.Group();
    const outerRing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.145, 0.16, 0.014, 48),
      new THREE.MeshStandardMaterial({
        color: 0x161f2c,
        metalness: 0.42,
        roughness: 0.58,
      })
    );
    outerRing.position.y = -0.007;
    outerRing.receiveShadow = true;
    outerRing.castShadow = true;
    plateGroup.add(outerRing);

    // Precision measurement ring around base (0.5m and 0.85m radius)
    const createRing = (r, color, opacity) => {
      const pts = [];
      const segs = 96;
      for (let i = 0; i <= segs; i++) {
        const a = (i / segs) * Math.PI * 2;
        pts.push(new THREE.Vector3(Math.cos(a) * r, 0.0015, Math.sin(a) * r));
      }
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity });
      return new THREE.Line(geo, mat);
    };
    plateGroup.add(createRing(0.45, 0x38bdf8, 0.25));
    plateGroup.add(createRing(0.85, 0x38bdf8, 0.35));

    // Base world coordinate axes (ROS Z-up visual indicator placed neatly on base)
    this.baseAxes = new THREE.AxesHelper(0.24);
    this.baseAxes.position.y = 0.002;
    plateGroup.add(this.baseAxes);

    this.stageGroup.add(plateGroup);
  }

  _setupIkGizmo() {
    this.ikTarget = new THREE.Group();
    this.ikTarget.name = 'IK_Target';
    this.ikTarget.visible = false;

    // Glowing holographic target sphere + axes
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 20, 20),
      new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.85 })
    );
    const halo = new THREE.Mesh(
      new THREE.RingGeometry(0.028, 0.035, 32),
      new THREE.MeshBasicMaterial({
        color: 0x38bdf8,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.6,
      })
    );
    const targetAxes = new THREE.AxesHelper(0.09);
    this.ikTarget.add(core, halo, targetAxes);
    this.scene.add(this.ikTarget);

    this.transformControls = new TransformControls(this.camera, this.renderer.domElement);
    this.transformControls.setSize(0.75);
    this.transformControls.setSpace('world');
    this.transformControls.attach(this.ikTarget);

    // Three.js r169+ uses getHelper() for adding TransformControls to scene
    this.transformHelper = typeof this.transformControls.getHelper === 'function'
      ? this.transformControls.getHelper()
      : this.transformControls;
    this.transformHelper.visible = false;
    this.transformControls.enabled = false;
    this.scene.add(this.transformHelper);

    this.isDraggingGizmo = false;
    this.onIkDragCallback = null;

    this.transformControls.addEventListener('dragging-changed', (e) => {
      this.controls.enabled = !e.value;
      this.isDraggingGizmo = e.value;
    });

    this.transformControls.addEventListener('objectChange', () => {
      if (this.isDraggingGizmo && this.onIkDragCallback) {
        this.onIkDragCallback(this.ikTarget);
      }
    });
  }

  setIkGizmoVisible(visible) {
    this.ikTarget.visible = visible;
    this.transformHelper.visible = visible;
    this.transformControls.enabled = visible;
  }

  setIkGizmoMode(mode) {
    this.transformControls.setMode(mode); // 'translate' | 'rotate'
  }

  _setupTrail() {
    this.trailEnabled = true;
    this.maxTrailPoints = 450;
    this.trailPoints = [];
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(this.maxTrailPoints * 3);
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setDrawRange(0, 0);

    const mat = new THREE.LineBasicMaterial({
      color: 0x22d3ee,
      transparent: true,
      opacity: 0.85,
    });
    this.trailLine = new THREE.Line(geo, mat);
    this.trailLine.frustumCulled = false;
    this.scene.add(this.trailLine);
  }

  pushTrailPoint(worldPos) {
    if (!this.trailEnabled) return;
    const last = this.trailPoints[this.trailPoints.length - 1];
    if (last && last.distanceToSquared(worldPos) < 0.000016) return; // > 4mm threshold

    this.trailPoints.push(worldPos.clone());
    if (this.trailPoints.length > this.maxTrailPoints) {
      this.trailPoints.shift();
    }

    const posAttr = this.trailLine.geometry.attributes.position;
    for (let i = 0; i < this.trailPoints.length; i++) {
      const p = this.trailPoints[i];
      posAttr.setXYZ(i, p.x, p.y, p.z);
    }
    posAttr.needsUpdate = true;
    this.trailLine.geometry.setDrawRange(0, this.trailPoints.length);
  }

  clearTrail() {
    this.trailPoints = [];
    this.trailLine.geometry.setDrawRange(0, 0);
  }

  setTrailEnabled(enabled) {
    this.trailEnabled = enabled;
    this.trailLine.visible = enabled;
    if (!enabled) this.clearTrail();
  }

  _setupWorkspaceEnvelope() {
    const geo = new THREE.SphereGeometry(0.85, 32, 24);
    const mat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      wireframe: true,
      transparent: true,
      opacity: 0.08,
      depthWrite: false,
    });
    this.envelopeMesh = new THREE.Mesh(geo, mat);
    this.envelopeMesh.position.set(0, 0.1625, 0);
    this.envelopeMesh.visible = false;
    this.scene.add(this.envelopeMesh);
  }

  setEnvelopeVisible(visible, radius = 0.85, centerY = 0.1625) {
    this.envelopeMesh.visible = visible;
    this.envelopeMesh.scale.setScalar(radius / 0.85);
    this.envelopeMesh.position.y = centerY;
  }

  setTheme(isLight) {
    const bg = isLight ? 0xe8eef5 : 0x0b1018;
    this.scene.background.setHex(bg);
    this.scene.fog.color.setHex(bg);
    this.floorMesh.material.color.setHex(isLight ? 0xd9e2ec : 0x0f1622);
    this.gridMajor.material.opacity = isLight ? 0.35 : 0.7;
  }

  animateCameraTo(position, target, durationMs = 520) {
    this.camAnim = {
      startPos: this.camera.position.clone(),
      endPos: new THREE.Vector3(...position),
      startTarget: this.controls.target.clone(),
      endTarget: new THREE.Vector3(...target),
      startTime: performance.now(),
      duration: durationMs,
    };
  }

  frameObject(object3D) {
    const box = new THREE.Box3().setFromObject(object3D);
    if (box.isEmpty()) return;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z, 0.4);
    const dist = maxDim * 1.95;
    this.animateCameraTo(
      [center.x + dist * 0.78, center.y + dist * 0.56, center.z + dist * 0.85],
      [center.x, center.y, center.z],
      600
    );
  }

  setCameraPreset(preset) {
    const t = [0, 0.42, 0];
    switch (preset) {
      case 'iso':
        this.animateCameraTo([1.25, 1.05, 1.45], t);
        break;
      case 'front':
        this.animateCameraTo([0, 0.52, 1.85], t);
        break;
      case 'side':
        this.animateCameraTo([1.85, 0.52, 0], t);
        break;
      case 'top':
        this.animateCameraTo([0.01, 2.25, 0.01], [0, 0.2, 0]);
        break;
    }
  }

  resize() {
    const w = this.container.clientWidth || window.innerWidth;
    const h = this.container.clientHeight || window.innerHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  update() {
    if (this.camAnim) {
      const now = performance.now();
      const rawT = Math.min(1, (now - this.camAnim.startTime) / this.camAnim.duration);
      // Smooth cubic ease-out
      const t = 1 - Math.pow(1 - rawT, 3);
      this.camera.position.lerpVectors(this.camAnim.startPos, this.camAnim.endPos, t);
      this.controls.target.lerpVectors(this.camAnim.startTarget, this.camAnim.endTarget, t);
      if (rawT >= 1) this.camAnim = null;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

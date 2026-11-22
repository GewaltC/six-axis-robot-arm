import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { parseURDF, createRobotHandle, enhanceRobotMaterial } from './urdfParser.js';

const ACTIVE_BLOB_URLS = new Set();

function trackBlobUrl(blob) {
  const url = URL.createObjectURL(blob);
  ACTIVE_BLOB_URLS.add(url);
  return url;
}

export function revokeImportedBlobUrls() {
  for (const u of ACTIVE_BLOB_URLS) {
    URL.revokeObjectURL(u);
  }
  ACTIVE_BLOB_URLS.clear();
}

function getExt(name) {
  const parts = (name || '').toLowerCase().split('.');
  return parts.length > 1 ? parts.pop() : '';
}

/**
 * Unpack a ZIP archive into an array of { path, file } entries using JSZip from CDN.
 */
async function unpackZip(zipFile) {
  const mod = await import('https://cdn.jsdelivr.net/npm/jszip@3.10.1/+esm');
  const JSZip = mod.default || mod;
  const zip = await JSZip.loadAsync(zipFile);
  const entries = [];
  const tasks = [];

  zip.forEach((relativePath, zipEntry) => {
    if (zipEntry.dir) return;
    if (relativePath.includes('__MACOSX') || relativePath.split('/').pop().startsWith('.')) return;
    tasks.push(
      zipEntry.async('blob').then((blob) => {
        entries.push({
          path: relativePath.replace(/\\/g, '/'),
          name: relativePath.split('/').pop(),
          blob,
        });
      })
    );
  });
  await Promise.all(tasks);
  return entries;
}

/**
 * Normalize raw FileList or DataTransfer files into uniform { path, name, blob } items.
 */
async function collectInputEntries(fileList) {
  const rawFiles = Array.from(fileList || []);
  const entries = [];

  for (const f of rawFiles) {
    const ext = getExt(f.name);
    if (ext === 'zip') {
      const zipEntries = await unpackZip(f);
      entries.push(...zipEntries);
    } else {
      entries.push({
        path: (f.webkitRelativePath || f.name).replace(/\\/g, '/'),
        name: f.name,
        blob: f,
      });
    }
  }
  return entries;
}

/**
 * Main entry point to import a robot model from user-provided files (URDF+meshes, ZIP, GLB/GLTF, DAE/STL/OBJ).
 */
export async function importRobotFromFiles(fileList, onProgress = null) {
  const entries = await collectInputEntries(fileList);
  if (entries.length === 0) {
    throw new Error('未检测到有效文件');
  }

  // 1. Check if a .urdf file exists among uploaded files/folder/zip
  const urdfEntry = entries.find((e) => getExt(e.name) === 'urdf');
  if (urdfEntry) {
    const urdfText = await urdfEntry.blob.text();
    const fileMap = new Map();
    for (const e of entries) {
      if (e === urdfEntry) continue;
      fileMap.set(e.path, trackBlobUrl(e.blob));
    }
    return await parseURDF(urdfText, { fileMap, onProgress });
  }

  // 2. Check for GLB / GLTF
  const gltfEntry = entries.find((e) => {
    const ext = getExt(e.name);
    return ext === 'glb' || ext === 'gltf';
  });
  if (gltfEntry) {
    return await importGltfRobot(gltfEntry, entries);
  }

  // 3. Fallback: Standalone DAE / STL / OBJ mesh model
  const meshEntry = entries.find((e) => ['dae', 'stl', 'obj'].includes(getExt(e.name)));
  if (meshEntry) {
    return await importSingleMeshRobot(meshEntry);
  }

  throw new Error('不支持的文件格式。请导入 .urdf(+网格)/.zip、.glb/.gltf、.dae、.stl 或 .obj 文件。');
}

/**
 * Load a GLB/GLTF scene, auto-fit scale to studio stage, auto-detect 6-axis joints,
 * and allow manual node-to-joint rebinding via bindJointSlot().
 */
async function importGltfRobot(gltfEntry, allEntries) {
  const manager = new THREE.LoadingManager();
  const extraUrls = new Map();
  for (const e of allEntries) {
    if (e !== gltfEntry) {
      extraUrls.set(e.name.toLowerCase(), trackBlobUrl(e.blob));
      extraUrls.set(e.path.toLowerCase(), extraUrls.get(e.name.toLowerCase()));
    }
  }
  manager.setURLModifier((url) => {
    const clean = url.split('/').pop().toLowerCase();
    return extraUrls.get(clean) || url;
  });

  const loader = new GLTFLoader(manager);
  const mainUrl = trackBlobUrl(gltfEntry.blob);
  const gltf = await loader.loadAsync(mainUrl);
  const modelScene = gltf.scene;

  modelScene.traverse((obj) => {
    if (obj.isMesh) {
      obj.castShadow = true;
      obj.receiveShadow = true;
    }
  });

  // Normalize scale and ground the imported GLTF onto y = 0
  const box = new THREE.Box3().setFromObject(modelScene);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 0.001);
  // If model is in mm (> 5m) or tiny (< 0.2m), normalize to ~1.0m height for comfortable viewing
  let scaleFactor = 1.0;
  if (maxDim > 4.0 || maxDim < 0.18) {
    scaleFactor = 1.0 / maxDim;
  }
  modelScene.scale.multiplyScalar(scaleFactor);
  modelScene.updateMatrixWorld(true);

  const scaledBox = new THREE.Box3().setFromObject(modelScene);
  const center = scaledBox.getCenter(new THREE.Vector3());
  modelScene.position.x -= center.x;
  modelScene.position.z -= center.z;
  modelScene.position.y -= scaledBox.min.y;
  modelScene.updateMatrixWorld(true);

  const robotRoot = new THREE.Group();
  robotRoot.name = `gltf_${gltfEntry.name}`;

  // Use ROS Z-up base frame wrapper so telemetry XYZ (mm, Z-up) stays consistent
  const baseFrame = new THREE.Group();
  baseFrame.name = 'ROS_Base_Frame';
  baseFrame.rotation.x = -Math.PI / 2;
  robotRoot.add(baseFrame);

  // Place GLTF Y-up content upright in Three.js world by placing directly under robotRoot
  robotRoot.add(modelScene);

  // Collect all named nodes with children or meshes for auto-detection & manual binding
  const allNamedNodes = [];
  modelScene.traverse((obj) => {
    if (obj === modelScene) return;
    if (obj.name && (obj.children.length > 0 || obj.isMesh || obj.isBone)) {
      allNamedNodes.push({
        name: obj.name,
        object: obj,
        baseQuat: obj.quaternion.clone(),
        type: obj.isBone ? 'Bone' : obj.isMesh ? 'Mesh' : 'Group',
      });
    }
  });

  // Heuristic 6-axis joint discovery from node hierarchy
  const detectedNodes = autoDetectSixAxisNodes(modelScene, allNamedNodes);

  const joints = detectedNodes.map((info, idx) =>
    createGltfJointWrapper(info.object, info.baseQuat, info.axis, idx, info.label)
  );

  // Place TCP marker at the end of the last detected joint or highest/furthest mesh tip
  const endEffector = new THREE.Group();
  endEffector.name = 'TCP_Frame';
  const tcpAxesHelper = new THREE.AxesHelper(0.11);
  const tcpDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.008, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x38bdf8 })
  );
  endEffector.add(tcpAxesHelper, tcpDot);

  if (joints.length > 0) {
    const lastObj = joints[joints.length - 1].rotGroup;
    lastObj.add(endEffector);
  } else {
    endEffector.position.set(0, scaledBox.max.y - scaledBox.min.y, 0);
    robotRoot.add(endEffector);
  }

  const handle = createRobotHandle({
    name: gltfEntry.name.replace(/\.(glb|gltf)$/i, ''),
    format: gltfEntry.name.toLowerCase().endsWith('.glb') ? 'GLB' : 'GLTF',
    root: robotRoot,
    baseFrame,
    joints,
    endEffector,
    tcpAxesHelper,
    isBuiltInUr5e: false,
    allNamedNodes,
  });

  // Allow user to dynamically rebind any joint slot (J1..J6) to any scene node + axis
  handle.rebindJoints = (bindings) => {
    // Restore previous rotations first
    for (const j of handle.joints) {
      if (j.baseQuat) j.rotGroup.quaternion.copy(j.baseQuat);
      if (j.axesHelper && j.axesHelper.parent) j.axesHelper.parent.remove(j.axesHelper);
    }

    const newJoints = [];
    bindings.forEach((b, idx) => {
      if (!b || !b.nodeName) return;
      const entry = allNamedNodes.find((n) => n.name === b.nodeName);
      if (!entry) return;
      const axisVec = axisLetterToVec(b.axis || 'Y');
      newJoints.push(createGltfJointWrapper(entry.object, entry.baseQuat, axisVec, idx, entry.name));
    });

    handle.joints = newJoints;
    if (newJoints.length > 0) {
      newJoints[newJoints.length - 1].rotGroup.add(handle.endEffector);
    }
    handle.root.updateMatrixWorld(true);
  };

  return handle;
}

function axisLetterToVec(letter) {
  switch ((letter || 'Y').toUpperCase()) {
    case 'X':
      return new THREE.Vector3(1, 0, 0);
    case 'Z':
      return new THREE.Vector3(0, 0, 1);
    case '-X':
      return new THREE.Vector3(-1, 0, 0);
    case '-Y':
      return new THREE.Vector3(0, -1, 0);
    case '-Z':
      return new THREE.Vector3(0, 0, -1);
    default:
      return new THREE.Vector3(0, 1, 0);
  }
}

function createGltfJointWrapper(targetObj, baseQuat, axisVec, index, label) {
  const axesHelper = new THREE.AxesHelper(0.08);
  axesHelper.visible = false;
  targetObj.add(axesHelper);

  const deltaQuat = new THREE.Quaternion();
  const axis = axisVec.clone().normalize();
  const axisChar = Math.abs(axis.x) > 0.5 ? 'X' : Math.abs(axis.z) > 0.5 ? 'Z' : 'Y';

  return {
    index,
    shortLabel: `J${index + 1}`,
    name: targetObj.name || `joint_${index + 1}`,
    label: label || targetObj.name || `Joint ${index + 1}`,
    type: 'revolute',
    axis,
    axisChar,
    lower: -Math.PI,
    upper: Math.PI,
    value: 0,
    baseQuat: baseQuat.clone(),
    rotGroup: targetObj,
    axesHelper,
    setValue(val) {
      const clamped = Math.max(this.lower, Math.min(this.upper, val));
      this.value = clamped;
      deltaQuat.setFromAxisAngle(this.axis, clamped);
      this.rotGroup.quaternion.copy(this.baseQuat).multiply(deltaQuat);
    },
  };
}

function autoDetectSixAxisNodes(modelScene, allNamedNodes) {
  // 1. Look for explicit joint/axis/link/bone naming patterns
  const patterns = [
    /(?:joint|j|axis|a|link|arm|wrist|shoulder|elbow)[_\-\s]?([1-6])/i,
    /(shoulder|upper_?arm|elbow|forearm|wrist_?1|wrist_?2|wrist_?3)/i,
  ];

  const matched = [];
  const used = new Set();

  for (const node of allNamedNodes) {
    const n = node.name.toLowerCase();
    if (n.includes('base') || n.includes('floor') || n.includes('camera') || n.includes('light')) continue;
    if (patterns.some((p) => p.test(n))) {
      if (!used.has(node.object)) {
        used.add(node.object);
        matched.push(node);
      }
    }
  }

  // If explicit naming gave 2..8 joints, take first 6 in hierarchy order
  let candidates = matched.slice(0, 6);

  // 2. Otherwise walk the deepest nested chain of parent->child groups (typical articulated arm hierarchy)
  if (candidates.length < 2) {
    let deepestChain = [];
    function walk(obj, chain) {
      const nextChain = obj === modelScene ? chain : [...chain, obj];
      if (nextChain.length > deepestChain.length) {
        deepestChain = nextChain;
      }
      for (const c of obj.children) {
        walk(c, nextChain);
      }
    }
    walk(modelScene, []);
    // Pick up to 6 internal pivot nodes along the chain
    const chainNodes = deepestChain
      .map((obj) => allNamedNodes.find((n) => n.object === obj))
      .filter(Boolean);
    candidates = chainNodes.slice(0, 6);
  }

  // Assign sensible default 6-axis rotation axes (J1=Y base yaw, J2=Z/X pitch, J3=Z/X pitch, J4=Y roll, J5=Z/X pitch, J6=Y flange roll)
  const defaultAxes = [
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 1, 0),
  ];

  return candidates.map((c, idx) => ({
    object: c.object,
    baseQuat: c.baseQuat,
    label: c.name,
    axis: defaultAxes[idx % defaultAxes.length],
  }));
}

async function importSingleMeshRobot(entry) {
  const ext = getExt(entry.name);
  const url = trackBlobUrl(entry.blob);
  let obj;

  if (ext === 'stl') {
    const geom = await new STLLoader().loadAsync(url);
    geom.computeVertexNormals();
    obj = new THREE.Mesh(
      geom,
      new THREE.MeshStandardMaterial({ color: 0xb8c4d4, metalness: 0.62, roughness: 0.32 })
    );
  } else if (ext === 'obj') {
    obj = await new OBJLoader().loadAsync(url);
  } else if (ext === 'dae') {
    const dae = await new ColladaLoader().loadAsync(url);
    obj = dae.scene;
  }

  obj.traverse((c) => {
    if (c.isMesh) {
      c.castShadow = true;
      c.receiveShadow = true;
      c.material = enhanceRobotMaterial(c.material);
    }
  });

  // Fit to 0.9m max size and rest on floor
  const box = new THREE.Box3().setFromObject(obj);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 0.001);
  if (maxDim > 3.5 || maxDim < 0.15) {
    obj.scale.multiplyScalar(0.9 / maxDim);
  }
  obj.updateMatrixWorld(true);
  const sBox = new THREE.Box3().setFromObject(obj);
  const center = sBox.getCenter(new THREE.Vector3());
  obj.position.set(-center.x, -sBox.min.y, -center.z);

  const robotRoot = new THREE.Group();
  const baseFrame = new THREE.Group();
  baseFrame.rotation.x = -Math.PI / 2;
  robotRoot.add(baseFrame, obj);

  const endEffector = new THREE.Group();
  endEffector.position.set(0, sBox.max.y - sBox.min.y, 0);
  const tcpAxesHelper = new THREE.AxesHelper(0.1);
  endEffector.add(tcpAxesHelper);
  robotRoot.add(endEffector);

  return createRobotHandle({
    name: entry.name.replace(/\.[^.]+$/, ''),
    format: ext.toUpperCase(),
    root: robotRoot,
    baseFrame,
    joints: [],
    endEffector,
    tcpAxesHelper,
  });
}

/**
 * Load an external STL/OBJ/GLB file as a custom end-effector tool mounted on the robot TCP flange.
 */
export async function loadEndEffectorTool(file) {
  const ext = getExt(file.name);
  const url = trackBlobUrl(file);
  let toolObj;

  if (ext === 'glb' || ext === 'gltf') {
    const gltf = await new GLTFLoader().loadAsync(url);
    toolObj = gltf.scene;
  } else if (ext === 'stl') {
    const geom = await new STLLoader().loadAsync(url);
    geom.computeVertexNormals();
    toolObj = new THREE.Mesh(
      geom,
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.65, roughness: 0.3 })
    );
  } else if (ext === 'obj') {
    toolObj = await new OBJLoader().loadAsync(url);
  } else {
    throw new Error('末端工具仅支持 .glb / .gltf / .stl / .obj 格式');
  }

  toolObj.traverse((c) => {
    if (c.isMesh) {
      c.castShadow = true;
      c.receiveShadow = true;
    }
  });

  // Auto-scale large millimeter tool CADs down to realistic ~140mm gripper size
  const box = new THREE.Box3().setFromObject(toolObj);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 0.001);
  if (maxDim > 0.45) {
    toolObj.scale.multiplyScalar(0.15 / maxDim);
  }
  toolObj.updateMatrixWorld(true);

  const wrapper = new THREE.Group();
  wrapper.name = 'Custom_EndEffector';
  wrapper.add(toolObj);
  return wrapper;
}

/**
 * Create a clean, precision procedural 2-finger parallel industrial electric gripper
 * (can be toggled ON/OFF on the UR5e ISO-9409-1-50-4-M6 flange, with controllable jaw opening!).
 */
export function createBuiltInGripper() {
  const group = new THREE.Group();
  group.name = 'BuiltIn_Electric_Gripper';

  const darkMetal = new THREE.MeshStandardMaterial({
    color: 0x262e3a,
    metalness: 0.72,
    roughness: 0.34,
  });
  const silverMetal = new THREE.MeshStandardMaterial({
    color: 0xd0d7e2,
    metalness: 0.85,
    roughness: 0.25,
  });
  const accentBlue = new THREE.MeshStandardMaterial({
    color: 0x38bdf8,
    metalness: 0.4,
    roughness: 0.35,
  });
  const rubberPad = new THREE.MeshStandardMaterial({
    color: 0x161a21,
    metalness: 0.1,
    roughness: 0.85,
  });

  // Adapter collar mounted flush on Z=0 flange extending along +Z (ROS tool0 Z+ forward)
  const collar = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.034, 0.018, 32), darkMetal);
  collar.rotation.x = Math.PI / 2;
  collar.position.z = 0.009;
  collar.castShadow = true;
  group.add(collar);

  // Accent status ring
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.0335, 0.0335, 0.004, 32), accentBlue);
  ring.rotation.x = Math.PI / 2;
  ring.position.z = 0.020;
  group.add(ring);

  // Actuator body
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.088, 0.044, 0.056), darkMetal);
  body.position.z = 0.050;
  body.castShadow = true;
  group.add(body);

  // Linear guide rail
  const rail = new THREE.Mesh(new THREE.BoxGeometry(0.082, 0.016, 0.010), silverMetal);
  rail.position.z = 0.082;
  group.add(rail);

  // Left & right sliding fingers
  const makeFinger = (side) => {
    const f = new THREE.Group();
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.026, 0.024), silverMetal);
    base.position.z = 0.095;
    base.castShadow = true;

    const tip = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.020, 0.048), silverMetal);
    tip.position.set(-side * 0.004, 0, 0.128);
    tip.castShadow = true;

    const pad = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.018, 0.032), rubberPad);
    pad.position.set(-side * 0.009, 0, 0.134);

    f.add(base, tip, pad);
    return f;
  };

  const leftFinger = makeFinger(-1);
  const rightFinger = makeFinger(1);
  group.add(leftFinger, rightFinger);

  // Set opening width [0..1] (0 = closed, 1 = fully open 85mm)
  const setOpening = (ratio) => {
    const r = Math.max(0, Math.min(1, ratio));
    const halfSpan = 0.011 + r * 0.026;
    leftFinger.position.x = -halfSpan;
    rightFinger.position.x = halfSpan;
  };
  setOpening(0.65);

  group.userData.setOpening = setOpening;
  return group;
}

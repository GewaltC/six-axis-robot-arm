import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

function parseVec3(str, fallback = [0, 0, 0]) {
  if (!str) return fallback.slice();
  const parts = str.trim().split(/\s+/).map(Number);
  return parts.length >= 3 && parts.every((n) => !Number.isNaN(n)) ? parts.slice(0, 3) : fallback.slice();
}

function applyUrdfOrigin(obj, originEl) {
  if (!originEl) return;
  const xyz = parseVec3(originEl.getAttribute('xyz'), [0, 0, 0]);
  const rpy = parseVec3(originEl.getAttribute('rpy'), [0, 0, 0]);
  obj.position.set(xyz[0], xyz[1], xyz[2]);
  // URDF RPY is fixed-axis (extrinsic) X -> Y -> Z, equivalent to intrinsic ZYX
  obj.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
}

/**
 * Upgrade Collada / legacy Phong materials to physically-based industrial PBR materials
 * while faithfully preserving original color schemes (including UR5e signature finishes).
 */
export function enhanceRobotMaterial(mat) {
  if (!mat) {
    return new THREE.MeshStandardMaterial({
      color: 0x94a3b8,
      metalness: 0.55,
      roughness: 0.35,
    });
  }
  if (Array.isArray(mat)) {
    return mat.map((m) => enhanceRobotMaterial(m));
  }

  const name = (mat.name || '').toLowerCase();
  const baseColor = mat.color ? mat.color.clone() : new THREE.Color(0x9ca3af);

  // Tuned industrial finishes for Universal Robots UR5e and generic CAD models
  let metalness = 0.45;
  let roughness = 0.38;

  if (name.includes('urblue') || name.includes('blue')) {
    // Anodized satin UR signature cyan-blue caps
    baseColor.setHex(0x4a9fd8);
    metalness = 0.35;
    roughness = 0.34;
  } else if (name.includes('linkgrey') || name.includes('silver') || name.includes('alum')) {
    // Brushed satin aluminum tubes & tool flange
    baseColor.setHex(0xcfd6df);
    metalness = 0.82;
    roughness = 0.28;
  } else if (name.includes('jointgrey') || name.includes('grey') || name.includes('gray')) {
    // Machined dark titanium-grey joint housings
    baseColor.setHex(0x5f6b7a);
    metalness = 0.62;
    roughness = 0.38;
  } else if (name.includes('black') || name.includes('dark') || name.includes('rubber')) {
    // Matte industrial anodized black rings & seals
    baseColor.setHex(0x1f252e);
    metalness = 0.25;
    roughness = 0.62;
  } else if (mat.isMeshStandardMaterial || mat.isMeshPhysicalMaterial) {
    return mat;
  }

  return new THREE.MeshStandardMaterial({
    name: mat.name,
    color: baseColor,
    map: mat.map || null,
    metalness,
    roughness,
    side: mat.side ?? THREE.FrontSide,
    transparent: mat.transparent || false,
    opacity: mat.opacity ?? 1.0,
  });
}

function resolveMeshUrl(rawFilename, baseUrl, fileMap) {
  if (!rawFilename) return null;
  let cleaned = rawFilename.trim().replace(/^package:\/\/[^/]+\//i, '').replace(/^\.\//, '');

  if (fileMap && fileMap.size > 0) {
    // 1. Exact relative match
    const normClean = cleaned.replace(/\\/g, '/').toLowerCase();
    for (const [key, blobUrl] of fileMap.entries()) {
      const normKey = key.replace(/\\/g, '/').toLowerCase();
      if (normKey === normClean || normKey.endsWith('/' + normClean)) {
        return { url: blobUrl, ext: getExtension(cleaned) };
      }
    }
    // 2. Basename fallback match
    const baseName = normClean.split('/').pop();
    for (const [key, blobUrl] of fileMap.entries()) {
      const keyBase = key.replace(/\\/g, '/').toLowerCase().split('/').pop();
      if (keyBase === baseName) {
        return { url: blobUrl, ext: getExtension(baseName) };
      }
    }
  }

  if (baseUrl) {
    const joined = new URL(cleaned, baseUrl).href;
    return { url: joined, ext: getExtension(cleaned) };
  }

  return null;
}

function getExtension(filename) {
  const parts = filename.toLowerCase().split('?')[0].split('.');
  return parts.length > 1 ? parts.pop() : '';
}

async function loadMeshAsset(url, ext, overrideMaterial) {
  if (ext === 'dae') {
    const loader = new ColladaLoader();
    const dae = await loader.loadAsync(url);
    const root = dae.scene;
    // URDF already operates in Z-up space; undo ColladaLoader's automatic Z_UP -> Y_UP flip
    root.rotation.set(0, 0, 0);
    root.updateMatrixWorld(true);

    // Strip any embedded cameras/lights and upgrade materials
    const toRemove = [];
    root.traverse((child) => {
      if (child.isLight || child.isCamera) {
        toRemove.push(child);
      } else if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
        child.material = overrideMaterial || enhanceRobotMaterial(child.material);
      }
    });
    toRemove.forEach((n) => n.parent?.remove(n));
    return root;
  }

  if (ext === 'stl') {
    const loader = new STLLoader();
    const geom = await loader.loadAsync(url);
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(
      geom,
      overrideMaterial ||
        new THREE.MeshStandardMaterial({
          color: 0xb8c4d4,
          metalness: 0.58,
          roughness: 0.34,
        })
    );
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  if (ext === 'obj') {
    const loader = new OBJLoader();
    const obj = await loader.loadAsync(url);
    obj.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
        child.material = overrideMaterial || enhanceRobotMaterial(child.material);
      }
    });
    return obj;
  }

  if (ext === 'glb' || ext === 'gltf') {
    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);
    gltf.scene.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    return gltf.scene;
  }

  throw new Error(`Unsupported mesh extension: .${ext}`);
}

function buildPrimitiveGeometry(geometryEl) {
  const boxEl = geometryEl.querySelector('box');
  if (boxEl) {
    const [x, y, z] = parseVec3(boxEl.getAttribute('size'), [0.1, 0.1, 0.1]);
    return new THREE.BoxGeometry(x, y, z);
  }
  const cylEl = geometryEl.querySelector('cylinder');
  if (cylEl) {
    const r = parseFloat(cylEl.getAttribute('radius') || '0.05');
    const l = parseFloat(cylEl.getAttribute('length') || '0.1');
    const g = new THREE.CylinderGeometry(r, r, l, 32);
    // URDF cylinders are along Z; Three.js cylinders are along Y
    g.rotateX(Math.PI / 2);
    return g;
  }
  const sphEl = geometryEl.querySelector('sphere');
  if (sphEl) {
    const r = parseFloat(sphEl.getAttribute('radius') || '0.05');
    return new THREE.SphereGeometry(r, 24, 24);
  }
  return null;
}

/**
 * Parse a URDF XML document into a Three.js kinematic robot structure.
 */
export async function parseURDF(urdfText, options = {}) {
  const { baseUrl = null, fileMap = null, onProgress = null } = options;

  const parser = new DOMParser();
  const xml = parser.parseFromString(urdfText, 'application/xml');
  const parseErr = xml.querySelector('parsererror');
  if (parseErr) {
    throw new Error('URDF XML 解析失败: ' + parseErr.textContent);
  }

  const robotEl = xml.querySelector('robot');
  if (!robotEl) {
    throw new Error('无效的 URDF 文件：缺少 <robot> 根节点');
  }

  const robotName = robotEl.getAttribute('name') || 'URDF Robot';

  // Global named materials
  const namedMaterials = new Map();
  for (const matEl of robotEl.children) {
    if (matEl.tagName === 'material') {
      const mName = matEl.getAttribute('name');
      const colorEl = matEl.querySelector('color');
      if (mName && colorEl) {
        const rgba = colorEl.getAttribute('rgba').trim().split(/\s+/).map(Number);
        namedMaterials.set(
          mName,
          new THREE.MeshStandardMaterial({
            name: mName,
            color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
            opacity: rgba[3] ?? 1,
            transparent: (rgba[3] ?? 1) < 0.99,
            metalness: 0.48,
            roughness: 0.36,
          })
        );
      }
    }
  }

  // Create Three.js Groups for every <link>
  const linkGroups = new Map();
  const linkVisualTasks = [];

  for (const linkEl of robotEl.querySelectorAll(':scope > link')) {
    const name = linkEl.getAttribute('name');
    const linkGroup = new THREE.Group();
    linkGroup.name = name;
    linkGroups.set(name, linkGroup);

    for (const visEl of linkEl.querySelectorAll(':scope > visual')) {
      const visGroup = new THREE.Group();
      applyUrdfOrigin(visGroup, visEl.querySelector(':scope > origin'));
      linkGroup.add(visGroup);

      // Visual material override if defined inline
      let overrideMat = null;
      const matEl = visEl.querySelector(':scope > material');
      if (matEl) {
        const colorEl = matEl.querySelector('color');
        if (colorEl) {
          const rgba = colorEl.getAttribute('rgba').trim().split(/\s+/).map(Number);
          overrideMat = new THREE.MeshStandardMaterial({
            color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
            opacity: rgba[3] ?? 1,
            transparent: (rgba[3] ?? 1) < 0.99,
            metalness: 0.5,
            roughness: 0.36,
          });
        } else if (matEl.getAttribute('name') && namedMaterials.has(matEl.getAttribute('name'))) {
          overrideMat = namedMaterials.get(matEl.getAttribute('name')).clone();
        }
      }

      const geomEl = visEl.querySelector(':scope > geometry');
      if (!geomEl) continue;

      const meshEl = geomEl.querySelector('mesh');
      if (meshEl) {
        const filename = meshEl.getAttribute('filename');
        const scale = parseVec3(meshEl.getAttribute('scale'), [1, 1, 1]);
        linkVisualTasks.push(async () => {
          const resolved = resolveMeshUrl(filename, baseUrl, fileMap);
          if (!resolved) {
            console.warn(`[URDF] 找不到网格文件: ${filename}`);
            return;
          }
          const meshObj = await loadMeshAsset(resolved.url, resolved.ext, overrideMat);
          meshObj.scale.multiply(new THREE.Vector3(scale[0], scale[1], scale[2]));
          visGroup.add(meshObj);
        });
      } else {
        const primGeo = buildPrimitiveGeometry(geomEl);
        if (primGeo) {
          const mesh = new THREE.Mesh(
            primGeo,
            overrideMat ||
              new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.5, roughness: 0.35 })
          );
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          visGroup.add(mesh);
        }
      }
    }
  }

  // Parse <joint> elements and assemble kinematic hierarchy
  const childLinkSet = new Set();
  const jointRecords = [];
  const childrenByParent = new Map();

  for (const jointEl of robotEl.querySelectorAll(':scope > joint')) {
    const name = jointEl.getAttribute('name');
    const type = (jointEl.getAttribute('type') || 'fixed').toLowerCase();
    const parentName = jointEl.querySelector('parent')?.getAttribute('link');
    const childName = jointEl.querySelector('child')?.getAttribute('link');

    const parentGroup = linkGroups.get(parentName);
    const childGroup = linkGroups.get(childName);
    if (!parentGroup || !childGroup) continue;

    childLinkSet.add(childName);
    if (!childrenByParent.has(parentName)) childrenByParent.set(parentName, []);
    childrenByParent.get(parentName).push({ jointName: name, childName, type });

    const originGroup = new THREE.Group();
    originGroup.name = `joint_origin_${name}`;
    applyUrdfOrigin(originGroup, jointEl.querySelector(':scope > origin'));
    parentGroup.add(originGroup);

    const rotGroup = new THREE.Group();
    rotGroup.name = `joint_rot_${name}`;
    originGroup.add(rotGroup);
    rotGroup.add(childGroup);

    if (type === 'revolute' || type === 'continuous' || type === 'prismatic') {
      const axisArr = parseVec3(jointEl.querySelector(':scope > axis')?.getAttribute('xyz'), [0, 0, 1]);
      const axis = new THREE.Vector3(axisArr[0], axisArr[1], axisArr[2]).normalize();

      const limitEl = jointEl.querySelector(':scope > limit');
      let lower = type === 'continuous' ? -Math.PI * 2 : -Math.PI;
      let upper = type === 'continuous' ? Math.PI * 2 : Math.PI;
      if (limitEl) {
        if (limitEl.hasAttribute('lower')) lower = parseFloat(limitEl.getAttribute('lower'));
        if (limitEl.hasAttribute('upper')) upper = parseFloat(limitEl.getAttribute('upper'));
      }

      const axesHelper = new THREE.AxesHelper(0.09);
      axesHelper.visible = false;
      rotGroup.add(axesHelper);

      const jointObj = {
        name,
        label: name,
        type,
        axis,
        lower,
        upper,
        value: 0,
        parentName,
        childName,
        originGroup,
        rotGroup,
        axesHelper,
        setValue(val) {
          const clamped = type === 'continuous' ? val : Math.max(this.lower, Math.min(this.upper, val));
          this.value = clamped;
          if (type === 'prismatic') {
            this.rotGroup.position.copy(this.axis).multiplyScalar(clamped);
          } else {
            this.rotGroup.quaternion.setFromAxisAngle(this.axis, clamped);
          }
        },
      };
      jointRecords.push(jointObj);
    }
  }

  // Find root link (no parent joint)
  let rootLinkName = null;
  for (const linkName of linkGroups.keys()) {
    if (!childLinkSet.has(linkName)) {
      rootLinkName = linkName;
      break;
    }
  }
  const rootLinkGroup = linkGroups.get(rootLinkName) || linkGroups.values().next().value;

  // Order joints in topological chain order from rootLinkName
  const orderedJoints = [];
  const jointByName = new Map(jointRecords.map((j) => [j.name, j]));
  const visited = new Set();
  let deepestLink = rootLinkGroup;
  let maxDepth = -1;

  function dfs(linkName, depth) {
    if (visited.has(linkName)) return;
    visited.add(linkName);
    if (depth >= maxDepth) {
      maxDepth = depth;
      deepestLink = linkGroups.get(linkName);
    }
    const edges = childrenByParent.get(linkName) || [];
    for (const edge of edges) {
      const j = jointByName.get(edge.jointName);
      if (j) orderedJoints.push(j);
      dfs(edge.childName, depth + 1);
    }
  }
  if (rootLinkName) dfs(rootLinkName, 0);

  // Assign friendly J1..Jn display labels
  orderedJoints.forEach((j, idx) => {
    j.index = idx;
    j.shortLabel = `J${idx + 1}`;
  });

  // Load all visual meshes in parallel with progress reporting
  let loadedCount = 0;
  const totalCount = linkVisualTasks.length;
  await Promise.all(
    linkVisualTasks.map(async (task) => {
      await task();
      loadedCount++;
      if (onProgress) onProgress(loadedCount, totalCount);
    })
  );

  // Determine End Effector (TCP): prefer 'tool0' or 'flange' or 'ee_link' or deepest chain link
  const tcpLink =
    linkGroups.get('tool0') ||
    linkGroups.get('ee_link') ||
    linkGroups.get('flange') ||
    deepestLink ||
    rootLinkGroup;

  const endEffector = new THREE.Group();
  endEffector.name = 'TCP_Frame';
  tcpLink.add(endEffector);

  // Visible TCP coordinate triad + subtle indicator dot
  const tcpAxesHelper = new THREE.AxesHelper(0.11);
  const tcpTipDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.008, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x38bdf8 })
  );
  endEffector.add(tcpAxesHelper, tcpTipDot);

  // Wrap in ROS Z-up -> Three.js Y-up root container
  const robotRoot = new THREE.Group();
  robotRoot.name = `robot_${robotName}`;

  const baseFrame = new THREE.Group();
  baseFrame.name = 'ROS_Base_Frame';
  baseFrame.rotation.x = -Math.PI / 2;
  baseFrame.add(rootLinkGroup);
  robotRoot.add(baseFrame);

  robotRoot.updateMatrixWorld(true);

  return createRobotHandle({
    name: robotName,
    format: 'URDF',
    root: robotRoot,
    baseFrame,
    joints: orderedJoints,
    endEffector,
    tcpAxesHelper,
    isBuiltInUr5e: robotName.toLowerCase() === 'ur5e',
  });
}

export function createRobotHandle(config) {
  const {
    name,
    format,
    root,
    baseFrame,
    joints,
    endEffector,
    tcpAxesHelper,
    isBuiltInUr5e = false,
    allNamedNodes = [],
  } = config;

  return {
    name,
    format,
    root,
    baseFrame,
    joints,
    endEffector,
    tcpAxesHelper,
    isBuiltInUr5e,
    allNamedNodes,

    getJointValues() {
      return this.joints.map((j) => j.value);
    },

    setJointValues(values) {
      for (let i = 0; i < this.joints.length && i < values.length; i++) {
        this.joints[i].setValue(values[i]);
      }
      this.root.updateMatrixWorld(true);
    },

    setJointsVisible(visible) {
      for (const j of this.joints) {
        if (j.axesHelper) j.axesHelper.visible = visible;
      }
    },

    setTcpVisible(visible) {
      if (this.tcpAxesHelper) this.tcpAxesHelper.visible = visible;
    },

    setWireframe(enabled) {
      this.root.traverse((obj) => {
        if (obj.isMesh && obj.material) {
          const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
          mats.forEach((m) => {
            if ('wireframe' in m) m.wireframe = enabled;
          });
        }
      });
    },

    dispose() {
      this.root.traverse((obj) => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) {
          const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
          mats.forEach((m) => m.dispose());
        }
      });
    },
  };
}

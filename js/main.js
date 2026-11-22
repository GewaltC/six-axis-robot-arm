import * as THREE from 'three';
import { RobotStudioScene } from './scene.js';
import { parseURDF } from './urdfParser.js';
import {
  importRobotFromFiles,
  loadEndEffectorTool,
  createBuiltInGripper,
  revokeImportedBlobUrls,
} from './modelImporter.js';
import { getTcpPose, basePoseToWorld, solveIK } from './kinematics.js';
import {
  UR5E_PRESETS,
  UR5E_DEMO_SEQUENCE,
  MotionController,
} from './presetsAndTrajectory.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

// ---- App State ----
const state = {
  studio: null,
  robot: null,
  motion: new MotionController(),
  builtInGripper: null,
  customTool: null,
  gripperRatio: 0.65,
  ikGizmoEnabled: false,
  showJointAxes: false,
  showEnvelope: false,
  wireframe: false,
  lightTheme: false,
  waypoints: [],
  jointUiRefs: [],
};

// ---- DOM Helpers ----
const $ = (id) => document.getElementById(id);

let toastTimer = null;
function showToast(msg, duration = 2600) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), duration);
}

function setLoading(visible, text = '正在加载模型...') {
  const ov = $('loadingOverlay');
  $('loadingText').textContent = text;
  ov.classList.toggle('hidden', !visible);
}

function setStatusPill(text, busy = false) {
  const pill = $('statusPill');
  pill.textContent = text;
  pill.classList.toggle('busy', busy);
}

// ---- Load Default Open-Source UR5e Model ----
async function loadDefaultUR5e() {
  setLoading(true, '正在加载开源精细模型 Universal Robots UR5e (CAD DAE)...');
  state.motion.stopAll();

  try {
    const urdfUrl = new URL('./models/ur5e/ur5e.urdf', window.location.href).href;
    const res = await fetch(urdfUrl);
    if (!res.ok) throw new Error(`无法读取 ${urdfUrl} (HTTP ${res.status})`);
    const urdfText = await res.text();

    const robot = await parseURDF(urdfText, {
      baseUrl: urdfUrl,
      onProgress: (done, total) => {
        setLoading(true, `正在加载 UR5e 精细网格 (${done}/${total})...`);
      },
    });

    mountRobot(robot, 'UR5e · 6-DOF · URDF');

    // Set stately industrial Ready stance on startup
    const ready = UR5E_PRESETS[0];
    robot.setJointValues(ready.deg.map((d) => d * DEG2RAD));
    state.gripperRatio = ready.gripper;
    $('sliderGripper').value = Math.round(ready.gripper * 100);
    updateGripperOpening(ready.gripper);

    syncJointSlidersFromRobot();
    syncTelemetryAndGizmo(true);
    state.studio.clearTrail();
    state.studio.setCameraPreset('iso');

    showToast('已加载开源精细模型：Universal Robots UR5e (BSD-3-Clause)');
  } catch (err) {
    console.error(err);
    showToast('默认模型加载失败：' + err.message, 4500);
  } finally {
    setLoading(false);
  }
}

// ---- Mount / Replace Active Robot in Scene ----
function mountRobot(newRobot, badgeLabel) {
  if (state.robot) {
    state.studio.scene.remove(state.robot.root);
    state.robot.dispose();
  }
  state.robot = newRobot;
  state.studio.scene.add(newRobot.root);

  // Attach built-in electric gripper cleanly to TCP
  state.builtInGripper = createBuiltInGripper();
  state.builtInGripper.visible = $('chkGripper').checked;
  state.builtInGripper.userData.setOpening(state.gripperRatio);
  newRobot.endEffector.add(state.builtInGripper);

  // Re-clear any previous custom tool
  state.customTool = null;
  $('btnRemoveTool').style.display = 'none';

  // Apply current display toggles
  newRobot.setJointsVisible(state.showJointAxes);
  newRobot.setWireframe(state.wireframe);

  // Update badges & conditional UI
  const dof = newRobot.joints.length;
  $('modelBadgeText').textContent = badgeLabel || `${newRobot.name} · ${dof}-DOF · ${newRobot.format}`;
  $('btnOpenMapper').style.display = newRobot.rebindJoints ? 'inline-flex' : 'none';
  $('presetsCard').style.display = newRobot.isBuiltInUr5e ? 'flex' : 'none';

  // Rebuild joint sliders
  buildJointControlsUI();
  state.studio.clearTrail();
  syncTelemetryAndGizmo(true);
}

// ---- Build J1..Jn Joint Controls UI ----
function buildJointControlsUI() {
  const list = $('jointList');
  list.innerHTML = '';
  state.jointUiRefs = [];

  if (!state.robot || state.robot.joints.length === 0) {
    list.innerHTML = `<div class="empty-hint">
      当前模型未检测到可动关节。${
        state.robot?.rebindJoints
          ? '<br/>请点击顶部 <b>「关节映射」</b> 手动绑定 J1–J6 节点。'
          : ''
      }
    </div>`;
    return;
  }

  state.robot.joints.forEach((joint, idx) => {
    const isPrismatic = joint.type === 'prismatic';
    const minVal = isPrismatic ? joint.lower * 1000 : Math.round(joint.lower * RAD2DEG);
    const maxVal = isPrismatic ? joint.upper * 1000 : Math.round(joint.upper * RAD2DEG);
    const curVal = isPrismatic ? joint.value * 1000 : joint.value * RAD2DEG;
    const unit = isPrismatic ? 'mm' : '°';
    const step = isPrismatic ? 1 : 0.5;
    const jogStep = isPrismatic ? 10 : 5;

    const item = document.createElement('div');
    item.className = 'joint-item';
    item.innerHTML = `
      <div class="joint-top">
        <div class="joint-name-wrap">
          <span class="joint-tag">${joint.shortLabel}</span>
          <span class="joint-subname" title="${joint.name}">${joint.name}</span>
        </div>
        <div class="joint-controls-right">
          <button class="jog-mini" data-dir="-1" title="点动 -${jogStep}${unit}">−</button>
          <input class="num-input" type="number" step="${step}" min="${minVal}" max="${maxVal}" value="${curVal.toFixed(1)}" />
          <span class="unit-label">${unit}</span>
          <button class="jog-mini" data-dir="1" title="点动 +${jogStep}${unit}">+</button>
        </div>
      </div>
      <input type="range" min="${minVal}" max="${maxVal}" step="${step}" value="${curVal}" />
      <div class="joint-limits">
        <span>${minVal}${unit}</span>
        <span>${maxVal}${unit}</span>
      </div>
    `;

    const rangeEl = item.querySelector('input[type="range"]');
    const numEl = item.querySelector('.num-input');
    const [btnMinus, btnPlus] = item.querySelectorAll('.jog-mini');

    const applyUserValue = (displayVal) => {
      state.motion.stopAll();
      const clamped = Math.max(minVal, Math.min(maxVal, displayVal));
      const radOrM = isPrismatic ? clamped / 1000 : clamped * DEG2RAD;
      joint.setValue(radOrM);
      state.robot.root.updateMatrixWorld(true);
      rangeEl.value = clamped;
      numEl.value = clamped.toFixed(1);
      syncTelemetryAndGizmo(true);
    };

    rangeEl.addEventListener('input', () => applyUserValue(parseFloat(rangeEl.value)));
    numEl.addEventListener('change', () => {
      const v = parseFloat(numEl.value);
      if (!Number.isNaN(v)) applyUserValue(v);
    });
    btnMinus.addEventListener('click', () => applyUserValue(parseFloat(rangeEl.value) - jogStep));
    btnPlus.addEventListener('click', () => applyUserValue(parseFloat(rangeEl.value) + jogStep));

    list.appendChild(item);
    state.jointUiRefs.push({ joint, rangeEl, numEl, isPrismatic });
  });
}

function syncJointSlidersFromRobot() {
  for (const ref of state.jointUiRefs) {
    const val = ref.isPrismatic ? ref.joint.value * 1000 : ref.joint.value * RAD2DEG;
    ref.rangeEl.value = val;
    ref.numEl.value = val.toFixed(1);
  }
}

function fmt1(v) {
  return (Math.abs(v) < 0.05 ? 0 : v).toFixed(1);
}

function syncTelemetryAndGizmo(syncGizmoPose = false) {
  if (!state.robot) return;
  const pose = getTcpPose(state.robot);

  $('telX').textContent = fmt1(pose.xMm);
  $('telY').textContent = fmt1(pose.yMm);
  $('telZ').textContent = fmt1(pose.zMm);
  $('telRX').textContent = `${fmt1(pose.rxDeg)}°`;
  $('telRY').textContent = `${fmt1(pose.ryDeg)}°`;
  $('telRZ').textContent = `${fmt1(pose.rzDeg)}°`;

  state.studio.pushTrailPoint(pose.worldPos);

  if (syncGizmoPose && !state.studio.isDraggingGizmo) {
    state.studio.ikTarget.position.copy(pose.worldPos);
    state.studio.ikTarget.quaternion.copy(pose.worldQuat);
  }
}

function syncIkInputsFromCurrentPose() {
  if (!state.robot) return;
  const pose = getTcpPose(state.robot);
  $('ikX').value = fmt1(pose.xMm);
  $('ikY').value = fmt1(pose.yMm);
  $('ikZ').value = fmt1(pose.zMm);
  $('ikRX').value = fmt1(pose.rxDeg);
  $('ikRY').value = fmt1(pose.ryDeg);
  $('ikRZ').value = fmt1(pose.rzDeg);
}

// ---- Gripper & Custom End-Effector ----
function updateGripperOpening(ratio) {
  state.gripperRatio = Math.max(0, Math.min(1, ratio));
  const mm = Math.round(state.gripperRatio * 85);
  const pct = Math.round(state.gripperRatio * 100);
  $('gripperValText').textContent = `${mm} mm (${pct}%)`;
  if (state.builtInGripper?.userData?.setOpening) {
    state.builtInGripper.userData.setOpening(state.gripperRatio);
  }
}

// ---- IK Controls ----
function setIkGizmoState(enabled) {
  state.ikGizmoEnabled = enabled;
  state.studio.setIkGizmoVisible(enabled);
  $('btnToggleIkGizmo').classList.toggle('active', enabled);
  $('btnIkEnablePanel').classList.toggle('active', enabled);
  $('btnIkEnablePanel').textContent = enabled ? '关闭 3D 拖拽 Gizmo' : '开启 3D 拖拽 Gizmo';
  if (enabled) {
    syncTelemetryAndGizmo(true);
    syncIkInputsFromCurrentPose();
    showToast('已开启 3D 逆运动学控制柄：拖动箭头即可实时牵引机械臂');
  }
}

function solveAndAnimateToCartesian(targetPoseObj) {
  if (!state.robot || state.robot.joints.length === 0) {
    showToast('当前模型无可控关节，无法进行 IK 求解');
    return;
  }
  state.motion.stopAll();
  const startJoints = state.robot.getJointValues();
  const { worldPos, worldQuat } = basePoseToWorld(state.robot, targetPoseObj);
  const useOrientation = $('chkIkOrientation').checked;

  // Solve on robot state, capture target joints, then revert and smoothly animate!
  const res = solveIK(state.robot, worldPos, worldQuat, {
    maxIterations: 75,
    useOrientation,
  });
  const targetJoints = state.robot.getJointValues();
  state.robot.setJointValues(startJoints);

  state.motion.animateTo(startJoints, targetJoints, 0.72);
  if (res.posError > 0.015) {
    showToast(`目标点可能超出工作空间 (残差 ${(res.posError * 1000).toFixed(1)} mm)，已移动至最接近可达姿态`);
  } else {
    showToast(`IK 求解成功 (误差 ${(res.posError * 1000).toFixed(2)} mm)`);
  }
}

// ---- Waypoints Management ----
function renderWaypoints() {
  const list = $('waypointList');
  if (state.waypoints.length === 0) {
    list.innerHTML = '<div class="empty-hint">暂无自定义示教点位，调整好姿态后点击「+ 记录当前点位」</div>';
    return;
  }

  list.innerHTML = '';
  state.waypoints.forEach((wp, idx) => {
    const degSummary = wp.rad.map((r) => `${Math.round(r * RAD2DEG)}°`).join(', ');
    const el = document.createElement('div');
    el.className = 'waypoint-item';
    el.innerHTML = `
      <div class="wp-info">
        <div class="wp-title">#${idx + 1} ${wp.name}</div>
        <div class="wp-angles">${degSummary}</div>
      </div>
      <div class="row-flex">
        <button class="btn sm" data-act="go" title="平滑移动至此点位">前往</button>
        <button class="btn sm danger" data-act="del" title="删除点位">×</button>
      </div>
    `;
    el.querySelector('[data-act="go"]').addEventListener('click', () => {
      if (!state.robot) return;
      state.motion.animateTo(
        state.robot.getJointValues(),
        wp.rad,
        0.85,
        state.gripperRatio,
        wp.gripper ?? state.gripperRatio
      );
    });
    el.querySelector('[data-act="del"]').addEventListener('click', () => {
      state.waypoints.splice(idx, 1);
      renderWaypoints();
    });
    list.appendChild(el);
  });
}

// ---- Custom Model Import Handler ----
async function handleImportFiles(fileList) {
  if (!fileList || fileList.length === 0) return;
  closeModal('importModal');
  setLoading(true, '正在解析并导入自定义 3D 模型...');
  state.motion.stopAll();

  try {
    revokeImportedBlobUrls();
    const robot = await importRobotFromFiles(fileList, (done, total) => {
      setLoading(true, `正在加载模型网格 (${done}/${total})...`);
    });

    mountRobot(robot);
    state.studio.frameObject(robot.root);
    syncIkInputsFromCurrentPose();

    const dof = robot.joints.length;
    if (dof > 0) {
      showToast(`导入成功：${robot.name} (${robot.format}, 检测到 ${dof} 个关节)`);
    } else if (robot.rebindJoints) {
      showToast(`已导入 ${robot.name}，请使用「关节映射」手动绑定 6 轴节点`, 4000);
      openMapperModal();
    } else {
      showToast(`已导入静态网格模型：${robot.name} (${robot.format})`);
    }
  } catch (err) {
    console.error(err);
    showToast('模型导入失败：' + err.message, 4500);
  } finally {
    setLoading(false);
  }
}

// ---- GLTF Joint Mapper Modal ----
function openMapperModal() {
  if (!state.robot || !state.robot.rebindJoints) return;
  const rows = $('mapperRows');
  rows.innerHTML = '';

  const nodes = state.robot.allNamedNodes || [];
  const axisOptions = ['Y', 'Z', 'X', '-Y', '-Z', '-X'];

  for (let i = 0; i < 6; i++) {
    const existing = state.robot.joints[i];
    const row = document.createElement('div');
    row.className = 'mapper-row';

    const nodeOptionsHtml = [
      `<option value="">-- 未绑定 --</option>`,
      ...nodes.map(
        (n) =>
          `<option value="${n.name.replace(/"/g, '&quot;')}" ${
            existing?.name === n.name ? 'selected' : ''
          }>[${n.type}] ${n.name}</option>`
      ),
    ].join('');

    const axisOptionsHtml = axisOptions
      .map(
        (ax) => `<option value="${ax}" ${existing?.axisChar === ax ? 'selected' : ''}>轴 ${ax}</option>`
      )
      .join('');

    row.innerHTML = `
      <span class="joint-tag">J${i + 1}</span>
      <select class="map-node">${nodeOptionsHtml}</select>
      <select class="map-axis">${axisOptionsHtml}</select>
    `;
    rows.appendChild(row);
  }

  $('mapperModal').classList.add('open');
}

function applyJointMapping() {
  if (!state.robot || !state.robot.rebindJoints) return;
  const bindings = [];
  document.querySelectorAll('#mapperRows .mapper-row').forEach((row) => {
    const nodeName = row.querySelector('.map-node').value;
    const axis = row.querySelector('.map-axis').value;
    if (nodeName) bindings.push({ nodeName, axis });
  });

  state.robot.rebindJoints(bindings);
  buildJointControlsUI();
  syncTelemetryAndGizmo(true);
  $('modelBadgeText').textContent = `${state.robot.name} · ${state.robot.joints.length}-DOF · ${state.robot.format}`;
  closeModal('mapperModal');
  showToast(`已更新关节映射：成功绑定 ${state.robot.joints.length} 个可动关节`);
}

function closeModal(id) {
  $(id).classList.remove('open');
}

// ---- Wire UI Events ----
function bindEvents() {
  // Tabs
  document.querySelectorAll('.tab-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab-btn').forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach((p) => p.classList.remove('active'));
      btn.classList.add('active');
      $(btn.dataset.tab).classList.add('active');
      if (btn.dataset.tab === 'tab-ik') syncIkInputsFromCurrentPose();
    });
  });

  // Topbar actions
  $('btnOpenImport').addEventListener('click', () => $('importModal').classList.add('open'));
  $('btnCloseImport').addEventListener('click', () => closeModal('importModal'));
  $('btnCloseImport2').addEventListener('click', () => closeModal('importModal'));
  $('btnResetDefault').addEventListener('click', () => loadDefaultUR5e());
  $('btnOpenMapper').addEventListener('click', () => openMapperModal());
  $('btnCloseMapper').addEventListener('click', () => closeModal('mapperModal'));
  $('btnCloseMapper2').addEventListener('click', () => closeModal('mapperModal'));
  $('btnApplyMapping').addEventListener('click', () => applyJointMapping());

  // File pickers
  $('btnPickFiles').addEventListener('click', () => $('inputModelFiles').click());
  $('btnPickFolder').addEventListener('click', () => $('inputModelFolder').click());
  $('inputModelFiles').addEventListener('change', (e) => {
    handleImportFiles(e.target.files);
    e.target.value = '';
  });
  $('inputModelFolder').addEventListener('change', (e) => {
    handleImportFiles(e.target.files);
    e.target.value = '';
  });

  // Global drag-and-drop
  let dragDepth = 0;
  window.addEventListener('dragenter', (e) => {
    e.preventDefault();
    dragDepth++;
    $('dropOverlay').classList.add('visible');
  });
  window.addEventListener('dragover', (e) => e.preventDefault());
  window.addEventListener('dragleave', () => {
    dragDepth = Math.max(0, dragDepth - 1);
    if (dragDepth === 0) $('dropOverlay').classList.remove('visible');
  });
  window.addEventListener('drop', (e) => {
    e.preventDefault();
    dragDepth = 0;
    $('dropOverlay').classList.remove('visible');
    if (e.dataTransfer?.files?.length) {
      handleImportFiles(e.dataTransfer.files);
    }
  });

  // Viewport toggles
  $('btnToggleIkGizmo').addEventListener('click', () => setIkGizmoState(!state.ikGizmoEnabled));
  $('btnIkEnablePanel').addEventListener('click', () => setIkGizmoState(!state.ikGizmoEnabled));

  $('btnGizmoTranslate').addEventListener('click', () => {
    state.studio.setIkGizmoMode('translate');
    $('btnGizmoTranslate').classList.add('active');
    $('btnGizmoRotate').classList.remove('active');
  });
  $('btnGizmoRotate').addEventListener('click', () => {
    state.studio.setIkGizmoMode('rotate');
    $('btnGizmoRotate').classList.add('active');
    $('btnGizmoTranslate').classList.remove('active');
  });

  $('btnToggleTrail').addEventListener('click', () => {
    const next = !state.studio.trailEnabled;
    state.studio.setTrailEnabled(next);
    $('btnToggleTrail').classList.toggle('active', next);
  });
  $('btnClearTrail').addEventListener('click', () => {
    state.studio.clearTrail();
    showToast('已清除 3D 运动轨迹');
  });

  $('btnToggleAxes').addEventListener('click', () => {
    state.showJointAxes = !state.showJointAxes;
    $('btnToggleAxes').classList.toggle('active', state.showJointAxes);
    state.robot?.setJointsVisible(state.showJointAxes);
  });

  $('btnToggleEnvelope').addEventListener('click', () => {
    state.showEnvelope = !state.showEnvelope;
    $('btnToggleEnvelope').classList.toggle('active', state.showEnvelope);
    state.studio.setEnvelopeVisible(state.showEnvelope);
  });

  $('btnToggleWireframe').addEventListener('click', () => {
    state.wireframe = !state.wireframe;
    $('btnToggleWireframe').classList.toggle('active', state.wireframe);
    state.robot?.setWireframe(state.wireframe);
  });

  $('btnToggleTheme').addEventListener('click', () => {
    state.lightTheme = !state.lightTheme;
    document.body.classList.toggle('theme-light', state.lightTheme);
    state.studio.setTheme(state.lightTheme);
    $('btnToggleTheme').textContent = state.lightTheme ? '深色' : '浅色';
  });

  $('btnCollapsePanel').addEventListener('click', () => {
    $('sidePanel').classList.toggle('collapsed');
  });
  $('btnExpandPanel').addEventListener('click', () => {
    $('sidePanel').classList.remove('collapsed');
  });

  // Camera buttons
  document.querySelectorAll('[data-cam]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const mode = btn.dataset.cam;
      if (mode === 'fit' && state.robot) {
        state.studio.frameObject(state.robot.root);
      } else {
        state.studio.setCameraPreset(mode);
      }
    });
  });

  // Zero All Joints
  $('btnZeroAll').addEventListener('click', () => {
    if (!state.robot) return;
    const cur = state.robot.getJointValues();
    const zeros = cur.map(() => 0);
    state.motion.animateTo(cur, zeros, 0.8);
  });

  // Gripper controls
  $('chkGripper').addEventListener('change', (e) => {
    if (state.builtInGripper) {
      state.builtInGripper.visible = e.target.checked && !state.customTool;
    }
  });
  $('sliderGripper').addEventListener('input', (e) => {
    updateGripperOpening(parseFloat(e.target.value) / 100);
  });

  // Mount custom end-effector tool
  $('btnMountTool').addEventListener('click', () => $('inputToolFile').click());
  $('inputToolFile').addEventListener('change', async (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file || !state.robot) return;
    try {
      const tool = await loadEndEffectorTool(file);
      if (state.customTool) state.robot.endEffector.remove(state.customTool);
      state.customTool = tool;
      state.robot.endEffector.add(tool);
      if (state.builtInGripper) state.builtInGripper.visible = false;
      $('btnRemoveTool').style.display = 'inline-flex';
      showToast(`已挂载自定义末端工具：${file.name}`);
    } catch (err) {
      showToast('工具加载失败：' + err.message, 3500);
    }
  });
  $('btnRemoveTool').addEventListener('click', () => {
    if (state.customTool && state.robot) {
      state.robot.endEffector.remove(state.customTool);
      state.customTool = null;
    }
    if (state.builtInGripper) state.builtInGripper.visible = $('chkGripper').checked;
    $('btnRemoveTool').style.display = 'none';
    showToast('已卸载自定义工具');
  });

  // Built-in UR5e presets
  const presetGrid = $('presetGrid');
  UR5E_PRESETS.forEach((p) => {
    const b = document.createElement('button');
    b.className = 'btn sm';
    b.textContent = p.name;
    b.addEventListener('click', () => {
      if (!state.robot) return;
      state.motion.animateTo(
        state.robot.getJointValues(),
        p.deg.map((d) => d * DEG2RAD),
        0.85,
        state.gripperRatio,
        p.gripper
      );
    });
    presetGrid.appendChild(b);
  });

  // Cartesian IK form & Jog pad
  $('btnSyncCurPose').addEventListener('click', () => syncIkInputsFromCurrentPose());
  $('btnSolveIkMove').addEventListener('click', () => {
    solveAndAnimateToCartesian({
      xMm: parseFloat($('ikX').value) || 0,
      yMm: parseFloat($('ikY').value) || 0,
      zMm: parseFloat($('ikZ').value) || 0,
      rxDeg: parseFloat($('ikRX').value) || 0,
      ryDeg: parseFloat($('ikRY').value) || 0,
      rzDeg: parseFloat($('ikRZ').value) || 0,
    });
  });

  document.querySelectorAll('[data-jog]').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (!state.robot) return;
      const pose = getTcpPose(state.robot);
      const axis = btn.dataset.jog;
      const deltaMm = parseInt(btn.dataset.dir, 10) * 25;
      const nextPose = {
        xMm: pose.xMm + (axis === 'x' ? deltaMm : 0),
        yMm: pose.yMm + (axis === 'y' ? deltaMm : 0),
        zMm: pose.zMm + (axis === 'z' ? deltaMm : 0),
        rxDeg: pose.rxDeg,
        ryDeg: pose.ryDeg,
        rzDeg: pose.rzDeg,
      };
      solveAndAnimateToCartesian(nextPose);
    });
  });

  // Real-time 3D IK Gizmo Drag callback
  state.studio.onIkDragCallback = (targetObj) => {
    if (!state.robot || state.robot.joints.length === 0) return;
    state.motion.stopAll();
    setStatusPill('IK SOLVING', true);
    solveIK(state.robot, targetObj.position, targetObj.quaternion, {
      maxIterations: 22,
      useOrientation: $('chkIkOrientation').checked,
    });
    syncJointSlidersFromRobot();
    syncTelemetryAndGizmo(false);
    syncIkInputsFromCurrentPose();
  };

  // Teach & Demo playback
  $('btnRunDemo').addEventListener('click', () => {
    if (!state.robot) return;
    if (!state.robot.isBuiltInUr5e) {
      showToast('内置搬运 Demo 专为 UR5e 运动学设计，自定义模型请使用下方点位示教功能');
      return;
    }
    state.studio.clearTrail();
    state.motion.playSequence(
      UR5E_DEMO_SEQUENCE,
      state.robot.getJointValues(),
      state.gripperRatio,
      $('chkLoop').checked
    );
  });

  $('btnStopMotion').addEventListener('click', () => {
    state.motion.stopAll();
    setStatusPill('READY', false);
  });

  $('sliderSpeed').addEventListener('input', (e) => {
    const mult = parseInt(e.target.value, 10) / 100;
    state.motion.speedMultiplier = mult;
    $('speedValText').textContent = `${mult.toFixed(2)}x`;
  });

  $('btnAddWaypoint').addEventListener('click', () => {
    if (!state.robot || state.robot.joints.length === 0) return;
    const idx = state.waypoints.length + 1;
    state.waypoints.push({
      name: `点位 P${idx}`,
      rad: state.robot.getJointValues(),
      gripper: state.gripperRatio,
      duration: 1.0,
    });
    renderWaypoints();
    showToast(`已记录示教点位 P${idx}`);
  });

  $('btnPlayWaypoints').addEventListener('click', () => {
    if (!state.robot || state.waypoints.length < 2) {
      showToast('请至少记录 2 个示教点位后再播放轨迹');
      return;
    }
    state.motion.playSequence(
      state.waypoints,
      state.robot.getJointValues(),
      state.gripperRatio,
      $('chkLoop').checked
    );
  });

  $('btnClearWp').addEventListener('click', () => {
    state.waypoints = [];
    renderWaypoints();
  });

  $('btnExportWp').addEventListener('click', () => {
    if (state.waypoints.length === 0) {
      showToast('当前没有可导出的示教点位');
      return;
    }
    const blob = new Blob([JSON.stringify(state.waypoints, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${state.robot?.name || 'robot'}_waypoints.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  });

  $('btnImportWp').addEventListener('click', () => $('inputWpJson').click());
  $('inputWpJson').addEventListener('change', async (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    try {
      const data = JSON.parse(await file.text());
      if (!Array.isArray(data)) throw new Error('JSON 须为点位数组');
      state.waypoints = data;
      renderWaypoints();
      showToast(`已导入 ${data.length} 个示教点位`);
    } catch (err) {
      showToast('点位 JSON 解析失败：' + err.message);
    }
  });

  state.motion.onSequenceState = (playing, stepName) => {
    if (playing) {
      setStatusPill(`PLAY: ${stepName}`, true);
    } else {
      setStatusPill('READY', false);
    }
  };
}

// ---- Main Animation Loop ----
let lastFrameTime = performance.now();

function animate(now) {
  requestAnimationFrame(animate);
  const dt = Math.min(0.05, (now - lastFrameTime) / 1000);
  lastFrameTime = now;

  if (state.robot) {
    const step = state.motion.update(dt);
    if (step) {
      state.robot.setJointValues(step.joints);
      if (step.gripper !== null && step.gripper !== undefined) {
        updateGripperOpening(step.gripper);
        $('sliderGripper').value = Math.round(step.gripper * 100);
      }
      syncJointSlidersFromRobot();
      syncTelemetryAndGizmo(true);
    } else if (!state.studio.isDraggingGizmo && !state.motion.isBusy()) {
      if ($('statusPill').textContent === 'IK SOLVING') {
        setStatusPill('READY', false);
      }
    }
  }

  state.studio.update();
}

// ---- Bootstrap ----
function init() {
  state.studio = new RobotStudioScene($('viewport'));
  bindEvents();
  requestAnimationFrame(animate);
  loadDefaultUR5e();
}

init();

import * as THREE from 'three';

const _worldPos = new THREE.Vector3();
const _worldQuat = new THREE.Quaternion();
const _baseInvMat = new THREE.Matrix4();
const _baseInvQuat = new THREE.Quaternion();
const _relPos = new THREE.Vector3();
const _relQuat = new THREE.Quaternion();
const _euler = new THREE.Euler(0, 0, 0, 'ZYX');

/**
 * Compute current Forward Kinematics TCP pose in Robot Base Frame (ROS Z-up, mm & degrees)
 * and in Three.js World Space.
 */
export function getTcpPose(robot) {
  robot.root.updateMatrixWorld(true);
  robot.endEffector.getWorldPosition(_worldPos);
  robot.endEffector.getWorldQuaternion(_worldQuat);

  // Convert into robot.baseFrame local space (ROS Z-up)
  _baseInvMat.copy(robot.baseFrame.matrixWorld).invert();
  _relPos.copy(_worldPos).applyMatrix4(_baseInvMat);

  robot.baseFrame.getWorldQuaternion(_baseInvQuat).invert();
  _relQuat.copy(_baseInvQuat).multiply(_worldQuat);
  _euler.setFromQuaternion(_relQuat, 'ZYX');

  const rad2deg = 180 / Math.PI;
  return {
    worldPos: _worldPos.clone(),
    worldQuat: _worldQuat.clone(),
    xMm: _relPos.x * 1000,
    yMm: _relPos.y * 1000,
    zMm: _relPos.z * 1000,
    rxDeg: _euler.x * rad2deg,
    ryDeg: _euler.y * rad2deg,
    rzDeg: _euler.z * rad2deg,
  };
}

/**
 * Convert a target pose in Robot Base Frame (mm, degrees ZYX) into Three.js World Pos & Quat.
 */
export function basePoseToWorld(robot, { xMm, yMm, zMm, rxDeg, ryDeg, rzDeg }) {
  robot.root.updateMatrixWorld(true);
  const deg2rad = Math.PI / 180;
  const localPos = new THREE.Vector3(xMm / 1000, yMm / 1000, zMm / 1000);
  const worldPos = localPos.applyMatrix4(robot.baseFrame.matrixWorld);

  const localQuat = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rxDeg * deg2rad, ryDeg * deg2rad, rzDeg * deg2rad, 'ZYX')
  );
  const baseWorldQuat = new THREE.Quaternion();
  robot.baseFrame.getWorldQuaternion(baseWorldQuat);
  const worldQuat = baseWorldQuat.multiply(localQuat);

  return { worldPos, worldQuat };
}

/**
 * Solve small dense linear system A * x = b (m x m, m = 3 or 6) via Gaussian elimination with partial pivoting.
 */
function solveLinearSystem(A, b) {
  const m = b.length;
  const M = A.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < m; col++) {
    let pivotRow = col;
    let maxVal = Math.abs(M[col][col]);
    for (let r = col + 1; r < m; r++) {
      const v = Math.abs(M[r][col]);
      if (v > maxVal) {
        maxVal = v;
        pivotRow = r;
      }
    }
    if (maxVal < 1e-12) continue;
    if (pivotRow !== col) {
      const tmp = M[col];
      M[col] = M[pivotRow];
      M[pivotRow] = tmp;
    }

    const pivot = M[col][col];
    for (let r = col + 1; r < m; r++) {
      const factor = M[r][col] / pivot;
      for (let c = col; c <= m; c++) {
        M[r][c] -= factor * M[col][c];
      }
    }
  }

  const x = new Array(m).fill(0);
  for (let r = m - 1; r >= 0; r--) {
    let sum = M[r][m];
    for (let c = r + 1; c < m; c++) {
      sum -= M[r][c] * x[c];
    }
    x[r] = Math.abs(M[r][r]) > 1e-12 ? sum / M[r][r] : 0;
  }
  return x;
}

/**
 * Compute orientation error vector (rotation axis * angle) from currentQuat to targetQuat.
 */
function computeOrientationError(currentQuat, targetQuat, outVec) {
  const qErr = targetQuat.clone().multiply(currentQuat.clone().invert());
  if (qErr.w < 0) {
    qErr.set(-qErr.x, -qErr.y, -qErr.z, -qErr.w);
  }
  const sinHalf = Math.sqrt(qErr.x * qErr.x + qErr.y * qErr.y + qErr.z * qErr.z);
  if (sinHalf < 1e-7) {
    outVec.set(0, 0, 0);
    return 0;
  }
  const angle = 2 * Math.atan2(sinHalf, Math.min(1, qErr.w));
  const scale = angle / sinHalf;
  outVec.set(qErr.x * scale, qErr.y * scale, qErr.z * scale);
  return angle;
}

/**
 * Real-time Damped Least-Squares (Levenberg-Marquardt) Numerical 6-DOF Inverse Kinematics.
 * Works generically for any serial URDF or bound GLTF robot arm.
 */
export function solveIK(robot, targetWorldPos, targetWorldQuat = null, options = {}) {
  const joints = robot.joints;
  const n = joints.length;
  if (n === 0) return { converged: false, posError: Infinity, rotError: Infinity };

  const {
    maxIterations = 28,
    posTolerance = 0.0008, // 0.8mm
    rotTolerance = 0.008,  // ~0.45 deg
    damping = 0.06,
    maxStepRad = 0.22,
    useOrientation = Boolean(targetWorldQuat) && n >= 6,
    oriWeight = 0.45,
  } = options;

  const m = useOrientation ? 6 : 3;
  const curPos = new THREE.Vector3();
  const curQuat = new THREE.Quaternion();
  const posErr = new THREE.Vector3();
  const rotErr = new THREE.Vector3();
  const pivot = new THREE.Vector3();
  const axisWorld = new THREE.Vector3();
  const parentQuat = new THREE.Quaternion();
  const rVec = new THREE.Vector3();
  const crossVec = new THREE.Vector3();

  let lastPosErrNorm = Infinity;
  let lastRotErrNorm = 0;

  for (let iter = 0; iter < maxIterations; iter++) {
    robot.root.updateMatrixWorld(true);
    robot.endEffector.getWorldPosition(curPos);
    robot.endEffector.getWorldQuaternion(curQuat);

    posErr.subVectors(targetWorldPos, curPos);
    lastPosErrNorm = posErr.length();

    if (useOrientation) {
      lastRotErrNorm = computeOrientationError(curQuat, targetWorldQuat, rotErr);
    }

    if (lastPosErrNorm <= posTolerance && (!useOrientation || lastRotErrNorm <= rotTolerance)) {
      return { converged: true, posError: lastPosErrNorm, rotError: lastRotErrNorm };
    }

    // Clamp error vector per iteration to prevent overshoot when far from target
    if (lastPosErrNorm > 0.14) {
      posErr.multiplyScalar(0.14 / lastPosErrNorm);
    }
    if (useOrientation && lastRotErrNorm > 0.35) {
      rotErr.multiplyScalar(0.35 / lastRotErrNorm);
    }

    const e = useOrientation
      ? [
          posErr.x,
          posErr.y,
          posErr.z,
          rotErr.x * oriWeight,
          rotErr.y * oriWeight,
          rotErr.z * oriWeight,
        ]
      : [posErr.x, posErr.y, posErr.z];

    // Build m x n geometric Jacobian J
    const J = Array.from({ length: m }, () => new Array(n).fill(0));

    for (let j = 0; j < n; j++) {
      const joint = joints[j];
      joint.rotGroup.getWorldPosition(pivot);
      joint.rotGroup.getWorldQuaternion(parentQuat);
      axisWorld.copy(joint.axis).applyQuaternion(parentQuat).normalize();

      if (joint.type === 'prismatic') {
        J[0][j] = axisWorld.x;
        J[1][j] = axisWorld.y;
        J[2][j] = axisWorld.z;
      } else {
        rVec.subVectors(curPos, pivot);
        crossVec.crossVectors(axisWorld, rVec);
        J[0][j] = crossVec.x;
        J[1][j] = crossVec.y;
        J[2][j] = crossVec.z;
        if (useOrientation) {
          J[3][j] = axisWorld.x * oriWeight;
          J[4][j] = axisWorld.y * oriWeight;
          J[5][j] = axisWorld.z * oriWeight;
        }
      }
    }

    // Compute A = J * J^T + lambda^2 * I
    const lam2 = damping * damping;
    const A = Array.from({ length: m }, () => new Array(m).fill(0));
    for (let r = 0; r < m; r++) {
      for (let c = 0; c < m; c++) {
        let sum = 0;
        for (let k = 0; k < n; k++) {
          sum += J[r][k] * J[c][k];
        }
        A[r][c] = sum + (r === c ? lam2 : 0);
      }
    }

    // Solve A * y = e
    const y = solveLinearSystem(A, e);

    // Compute dTheta = J^T * y and apply clamped step
    for (let j = 0; j < n; j++) {
      let dq = 0;
      for (let r = 0; r < m; r++) {
        dq += J[r][j] * y[r];
      }
      if (Number.isNaN(dq)) continue;
      dq = Math.max(-maxStepRad, Math.min(maxStepRad, dq));
      joints[j].setValue(joints[j].value + dq);
    }
  }

  robot.root.updateMatrixWorld(true);
  return {
    converged: lastPosErrNorm <= posTolerance * 3,
    posError: lastPosErrNorm,
    rotError: lastRotErrNorm,
  };
}

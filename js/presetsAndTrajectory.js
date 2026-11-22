const DEG2RAD = Math.PI / 180;

export const UR5E_PRESETS = [
  {
    id: 'ready',
    name: '待命位 Ready',
    deg: [0, -78, 92, -104, -90, 0],
    gripper: 0.65,
  },
  {
    id: 'upright',
    name: '竖直零位 Upright',
    deg: [0, -90, 0, -90, 0, 0],
    gripper: 0.5,
  },
  {
    id: 'pickA',
    name: '抓取位 Pick A',
    deg: [42, -52, 86, -124, -90, 42],
    gripper: 0.15,
  },
  {
    id: 'placeB',
    name: '放置位 Place B',
    deg: [-48, -55, 92, -127, -90, -48],
    gripper: 0.85,
  },
  {
    id: 'inspect',
    name: '视觉巡检 Inspect',
    deg: [22, -108, 70, -52, -78, 30],
    gripper: 0.5,
  },
  {
    id: 'folded',
    name: '折叠收纳 Folded',
    deg: [0, -142, 146, -94, -90, 0],
    gripper: 0.0,
  },
];

export const UR5E_DEMO_SEQUENCE = [
  { name: '待命起始', rad: [0, -78, 92, -104, -90, 0].map((d) => d * DEG2RAD), gripper: 0.85, duration: 1.0 },
  { name: 'A区上方', rad: [42, -74, 80, -96, -90, 42].map((d) => d * DEG2RAD), gripper: 0.85, duration: 1.1 },
  { name: 'A区下降抓取', rad: [42, -50, 88, -128, -90, 42].map((d) => d * DEG2RAD), gripper: 0.85, duration: 0.9 },
  { name: '夹爪闭合', rad: [42, -50, 88, -128, -90, 42].map((d) => d * DEG2RAD), gripper: 0.12, duration: 0.5 },
  { name: 'A区抬升', rad: [42, -76, 78, -92, -90, 42].map((d) => d * DEG2RAD), gripper: 0.12, duration: 0.9 },
  { name: 'B区上方', rad: [-48, -76, 80, -94, -90, -48].map((d) => d * DEG2RAD), gripper: 0.12, duration: 1.3 },
  { name: 'B区下降放置', rad: [-48, -52, 90, -128, -90, -48].map((d) => d * DEG2RAD), gripper: 0.12, duration: 0.9 },
  { name: '夹爪释放', rad: [-48, -52, 90, -128, -90, -48].map((d) => d * DEG2RAD), gripper: 0.85, duration: 0.5 },
  { name: 'B区抬升复位', rad: [-48, -78, 82, -94, -90, -48].map((d) => d * DEG2RAD), gripper: 0.85, duration: 0.9 },
];

export class MotionController {
  constructor() {
    this.activeTransition = null;
    this.sequence = null;
    this.speedMultiplier = 1.0;
    this.onStep = null;
    this.onSequenceState = null;
  }

  /**
   * Smoothly animate joints (and optional gripper) from current values to target values.
   */
  animateTo(startJoints, targetJoints, durationSec = 0.85, startGripper = null, targetGripper = null) {
    this.stopSequence();
    this.activeTransition = {
      startJoints: startJoints.slice(),
      targetJoints: targetJoints.slice(),
      startGripper,
      targetGripper,
      elapsed: 0,
      duration: Math.max(0.05, durationSec),
    };
  }

  /**
   * Play a sequence of waypoints [{ name, rad, gripper, duration }, ...]
   */
  playSequence(waypoints, currentJoints, currentGripper, loop = true) {
    if (!waypoints || waypoints.length < 1) return;
    this.activeTransition = null;
    this.sequence = {
      waypoints: waypoints.map((w) => ({
        name: w.name,
        rad: w.rad.slice(),
        gripper: w.gripper ?? currentGripper,
        duration: w.duration ?? 1.1,
      })),
      loop,
      segIndex: 0,
      segElapsed: 0,
      fromJoints: currentJoints.slice(),
      fromGripper: currentGripper,
    };
    if (this.onSequenceState) this.onSequenceState(true, waypoints[0].name);
  }

  stopSequence() {
    const wasPlaying = Boolean(this.sequence);
    this.sequence = null;
    if (wasPlaying && this.onSequenceState) {
      this.onSequenceState(false, null);
    }
  }

  stopAll() {
    this.activeTransition = null;
    this.stopSequence();
  }

  isBusy() {
    return Boolean(this.activeTransition || this.sequence);
  }

  /**
   * Advance interpolation by dtSec. Returns { joints, gripper } if an update occurred, else null.
   */
  update(dtSec) {
    const scaledDt = dtSec * this.speedMultiplier;

    if (this.activeTransition) {
      const tr = this.activeTransition;
      tr.elapsed += scaledDt;
      const rawT = Math.min(1, tr.elapsed / tr.duration);
      // Smooth S-curve cosine easing (zero start/end velocity)
      const t = 0.5 - 0.5 * Math.cos(Math.PI * rawT);

      const joints = tr.startJoints.map((s, i) => {
        const tgt = tr.targetJoints[i] ?? s;
        return s + (tgt - s) * t;
      });
      const gripper =
        tr.startGripper !== null && tr.targetGripper !== null
          ? tr.startGripper + (tr.targetGripper - tr.startGripper) * t
          : null;

      if (rawT >= 1) {
        this.activeTransition = null;
      }
      return { joints, gripper };
    }

    if (this.sequence) {
      const seq = this.sequence;
      const targetWp = seq.waypoints[seq.segIndex];
      seq.segElapsed += scaledDt;
      const dur = Math.max(0.08, targetWp.duration);
      const rawT = Math.min(1, seq.segElapsed / dur);
      const t = 0.5 - 0.5 * Math.cos(Math.PI * rawT);

      const joints = seq.fromJoints.map((s, i) => {
        const tgt = targetWp.rad[i] ?? s;
        return s + (tgt - s) * t;
      });
      const gripper =
        seq.fromGripper !== null && targetWp.gripper !== null
          ? seq.fromGripper + (targetWp.gripper - seq.fromGripper) * t
          : null;

      if (rawT >= 1) {
        seq.fromJoints = targetWp.rad.slice();
        seq.fromGripper = targetWp.gripper;
        seq.segElapsed = 0;
        seq.segIndex++;

        if (seq.segIndex >= seq.waypoints.length) {
          if (seq.loop) {
            seq.segIndex = 0;
            if (this.onSequenceState) this.onSequenceState(true, seq.waypoints[0].name);
          } else {
            this.stopSequence();
          }
        } else {
          if (this.onSequenceState) this.onSequenceState(true, seq.waypoints[seq.segIndex].name);
        }
      }

      return { joints, gripper };
    }

    return null;
  }
}

(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MotorCore = api;
})(globalThis, function () {
  'use strict';
  const INITIAL_ANGLE_DEG = 12.77;
  const RAW_PER_AMP = 16384 / 20;
  const toAmp = raw => raw / RAW_PER_AMP;
  const toRaw = amp => Math.round(amp * RAW_PER_AMP);
  const scaleRaw = (raw, scale) => {
    const value = raw * scale;
    return Math.trunc(value + (value < 0 ? -.5 : .5));
  };

  function parseTelemetry(line) {
    const fields = line.trim().split(',');
    const minimum = fields[0] === 'V11' && fields.length === 108;
    const settling = minimum || ['V9', 'V10'].includes(fields[0]) && fields.length === 105;
    const position = settling || fields[0] === 'V8' && fields.length === 100;
    const directional = position || fields[0] === 'V7' && fields.length === 76;
    const timing = directional || fields[0] === 'V6' && fields.length === 58;
    const threshold = timing || fields[0] === 'V5' && fields.length === 55;
    const speed = threshold || fields[0] === 'V4' && fields.length === 52;
    const scaled = speed || fields[0] === 'V3' && fields.length === 28;
    if (!scaled && (fields.length !== 24 || fields[0] !== 'V2')) return null;
    if (!fields.slice(1).every(x => /^-?\d+$/.test(x))) return null;
    const n = fields.slice(1).map(Number);
    if (!n.every(Number.isSafeInteger) || n[1] < 0 || n[1] > (position ? 6 : speed ? 5 : 4) ||
        n[3] < 0 || n[3] > 3 || n[4] < 0 || n[4] > 7) return null;
    return {
      angleSettling: settling ? {toleranceDeg:n[99]/1000,holdMs:n[100],state:n.slice(101,104)} : null,
      angleTarget: position ? n.slice(75,78).map(v=>v/1000) : null,
      anglePid: position ? [0,1,2].map(i=>({
        up:{kp:n[78+6*i]/1000,ki:n[79+6*i]/1000,kd:n[80+6*i]/1000},
        down:{kp:n[81+6*i]/1000,ki:n[82+6*i]/1000,kd:n[83+6*i]/1000},
        maxSpeed:n[96+i]/1000,...(minimum?{minSpeed:n[104+i]/1000}:{})
      })) : null,
      requestedRaw: scaled ? n[26] : null,
      version: speed ? fields[0] : scaled ? 'V3' : 'V2',
      timing: timing ? {speedPeriodMs: n[54], filterPeriodMs: n[55], cutoffHz: n[56] / 1000} : null,
      frictionMove: threshold ? n.slice(51, 54).map(x => x / 1000) : null,
      speedMeasured: speed ? n.slice(27, 30).map(x => x / 1000) : null,
      speedTarget: speed ? n.slice(30, 33).map(x => x / 1000) : null,
      pidSettings: speed ? [0, 1, 2].map(i => ({ kp: n[33 + i * 4] / 1000, ki: n[34 + i * 4] / 1000, kd: n[35 + i * 4] / 1000, friction: n[36 + i * 4] })) : null,
      pidDirectional: directional ? [0, 1, 2].map(i => ({
        up: {kp: n[57 + 6*i]/1000, ki: n[58 + 6*i]/1000, kd: n[59 + 6*i]/1000},
        down: {kp: n[60 + 6*i]/1000, ki: n[61 + 6*i]/1000, kd: n[62 + 6*i]/1000}
      })) : null,
      pidOutput: speed ? n.slice(45, 48) : null, frictionOutput: speed ? n.slice(48, 51) : null,
      outputScale: scaled ? n.slice(23, 26).map(x => x / 1000) : null,
      ms: n[0], mode: n[1], kg: n[2] / 1000, motor: n[3], fresh: n[4],
      angles: n.slice(5, 8).map(x => x / 1000),
      pose: [n[8] / 1000, n[9] / 1000, n[10] / 1e6],
      base: n.slice(11, 14), delta: n.slice(14, 17),
      total: n.slice(17, 20), feedback: n.slice(20, 23)
    };
  }

  class LineReader {
    constructor(onLine) { this.buffer = ''; this.discard = false; this.onLine = onLine; }
    push(text) {
      for (const ch of text) {
        if (ch === '\n') {
          if (!this.discard) this.onLine(this.buffer.replace(/\r$/, ''));
          this.buffer = ''; this.discard = false;
        } else if (!this.discard) {
          this.buffer += ch;
          if (this.buffer.length > 1280) { this.buffer = ''; this.discard = true; }
        }
      }
    }
  }

  class BreakawayTest {
    constructor({ send, onFinish, now = () => performance.now() }) {
      this.send = send; this.onFinish = onFinish; this.now = now;
      this.run = null; this.latest = null; this.lastFrameAt = -Infinity;
    }
    accept(frame) {
      const now = this.now();
      this.latest = frame; this.lastFrameAt = now;
      const r = this.run;
      if (!r) return;
      if (frame.fresh !== 7) { this.finish('反馈中断'); return; }
      if (frame.mode === 0) { this.finish('设备已急停', false); return; }
      if (r.phase === 'arming') {
        if (frame.mode === 2 && frame.motor === r.motor && (frame.requestedRaw == null ? frame.delta[r.motor - 1] === 0 : frame.requestedRaw === 0)) {
          r.phase = 'ramping'; r.startedAt = now;
          r.startBase = toAmp(frame.base[r.motor - 1]);
        } else return;
      }
      if (frame.mode !== 2 || frame.motor !== r.motor) {
        this.finish('设备已撤销增量', false); return;
      }
      if (frame.ms === r.lastSampleMs) return;
      r.lastSampleMs = frame.ms;
      const movement = r.direction * (frame.angles[r.motor - 1] - r.startAngle);
      const sign = movement >= r.threshold ? 1 : movement <= -r.threshold ? -1 : 0;
      if (!sign) { r.confirmed = 0; r.candidate = null; return; }
      if (!r.candidate || r.candidate.sign !== sign) {
        r.confirmed = 0;
        r.candidate = { ...this.measurement(frame), sign };
      }
      r.confirmed++;
      if (r.confirmed >= 3) {
        this.finish(sign > 0 ? '检测到起动' : '检测到反向位移', true, r.candidate);
      }
    }
    start(motor, direction, options) {
      if (this.run) throw new Error('请先结束当前测试');
      if (!this.latest || this.now() - this.lastFrameAt > 500 || this.latest.fresh !== 7)
        throw new Error('等待新固件遥测及三路有效反馈');
      if (this.latest.mode !== 1) throw new Error('先点击“启用重力补偿”，建立基线');
      const { limit, ramp, threshold, duration } = options;
      if (![1, 2, 3].includes(motor) || ![-1, 1].includes(direction) ||
          !(limit >= .01 && limit <= 5) || !(ramp >= .01 && ramp <= 2) ||
          !(threshold >= .05 && threshold <= 10) || !(duration >= 1 && duration <= 60))
        throw new Error('请检查增量上限、速率、位移阈值和最长时间');
      this.run = {
        motor, direction, limit, ramp, threshold, duration,
        phase: 'arming', requestedAt: this.now(), lastSendAt: this.now(),
        startAngle: this.latest.angles[motor - 1],
        confirmed: 0, candidate: null, lastSampleMs: null, requestedDelta: 0
      };
      this.send(`D,${motor},${direction},0`);
    }
    tick() {
      const r = this.run, now = this.now();
      if (!r) return;
      if (now - this.lastFrameAt > 500) { this.finish('遥测超时'); return; }
      if (r.phase === 'arming' && now - r.requestedAt > 1500) {
        this.finish('设备未确认测试'); return;
      }
      const elapsed = r.phase === 'ramping' ? (now - r.startedAt) / 1000 : 0;
      if (elapsed >= r.duration) { this.finish('达到最长时间'); return; }
      if (now - r.lastSendAt >= 50) {
        r.requestedDelta = r.direction * Math.min(r.limit, r.ramp * elapsed);
        this.send(`D,${r.motor},${r.direction},${toRaw(r.requestedDelta)}`);
        r.lastSendAt = now;
      }
    }
    measurement(frame = this.latest) {
      const r = this.run, i = r.motor - 1;
      return {
        motor: r.motor, direction: r.direction,
        startAngle: r.startAngle ?? frame.angles[i], angle: frame.angles[i],
        baseline: toAmp(frame.base[i]), delta: toAmp(frame.delta[i]),
        total: toAmp(frame.total[i]), threshold: r.threshold,
        elapsed: r.startedAt === undefined ? 0 : (this.now() - r.startedAt) / 1000
      };
    }
    finish(reason = '手动结束', restore = true, measured = null) {
      if (!this.run) return;
      const result = { ...(measured || this.measurement()), reason };
      this.run = null;
      if (restore) this.send('G');
      this.onFinish(result);
    }
  }
  class FixedCurrentTest extends BreakawayTest {
    accept(frame) {
      this.latest = frame; this.lastFrameAt = this.now();
      const r = this.run;
      if (!r) return;
      if (frame.fresh !== 7) { this.finish('反馈中断'); return; }
      if (frame.mode === 0) { this.finish('设备已急停', false); return; }
      if (r.phase === 'arming') {
        const i = r.motor - 1;
        const feedforwardActive = r.gravityOn;
        const scale = frame.outputScale?.[i] ?? 1;
        const expected = Math.max(-16384, Math.min(16384, (feedforwardActive ? frame.base[i] : 0) + scaleRaw(r.raw, scale)));
        const confirmed = frame.requestedRaw == null ? frame.total[i] === expected : frame.requestedRaw === r.raw;
        if (frame.mode === (r.gravityOn ? 4 : 3) && frame.motor === r.motor &&
            (feedforwardActive || frame.base[i] === 0) && confirmed) {
          r.phase = 'holding'; r.startedAt = this.now();
        }
      } else if (frame.mode !== (r.gravityOn ? 4 : 3) || frame.motor !== r.motor) {
        this.finish('设备已结束固定指令', false);
      }
    }
    start(motor, raw, duration, gravityOn = false) {
      if (this.run) throw new Error('固定指令测试已在运行');
      if (!this.latest || this.now() - this.lastFrameAt > 500 || this.latest.fresh !== 7)
        throw new Error('等待新固件遥测及三路有效反馈');
      if (this.latest.mode !== 1) throw new Error('先结束当前模式并恢复普通补偿逻辑');
      if (![1, 2, 3].includes(motor)) throw new Error('请选择电机');
      this.validate(raw, duration);
      this.run = { kind: 'fixed', motor };
      this.update(raw, duration, gravityOn);
    }
    validate(raw, duration) {
      if (!Number.isInteger(raw) || raw < -16384 || raw > 16384)
        throw new Error('固定指令必须是 -16384 到 16384 的整数');
      if (!Number.isFinite(duration) || duration < 1 || duration > 60)
        throw new Error('固定指令持续时间为 1 到 60 秒');
    }
    update(raw, duration, gravityOn = this.run?.gravityOn ?? false) {
      this.validate(raw, duration);
      const r = this.run;
      if (!r) throw new Error('固定指令模式尚未启动');
      Object.assign(r, { raw, duration, gravityOn: Boolean(gravityOn), direction: raw < 0 ? -1 : 1,
        startAngle: this.latest.angles[r.motor - 1], phase: 'arming',
        requestedAt: this.now(), lastSendAt: this.now(), startedAt: undefined });
      this.send(`F,${r.motor},${r.gravityOn ? 1 : 0},${raw}`);
    }
    tick() {
      const r = this.run, now = this.now();
      if (!r) return;
      if (now - this.lastFrameAt > 500) { this.finish('遥测超时'); return; }
      if (r.phase === 'arming' && now - r.requestedAt > 1500) {
        this.finish('设备未确认固定指令，请更新固件'); return;
      }
      if (r.phase === 'holding' && now - r.startedAt >= r.duration * 1000) {
        this.finish('固定指令达到设定时间'); return;
      }
      if (now - r.lastSendAt >= 50) {
        this.send(`F,${r.motor},${r.gravityOn ? 1 : 0},${r.raw}`); r.lastSendAt = now;
      }
    }
    measurement(frame = this.latest) {
      return { ...super.measurement(frame), kind: 'fixed', requestedRaw: this.run.raw, gravityOn: this.run.gravityOn };
    }
  }
  class SpeedLoop {
    constructor({ send, onFinish, now = () => performance.now() }) {
      Object.assign(this, { send, onFinish, now });
      this.run = null; this.latest = null; this.lastFrameAt = -Infinity;
      this.mode = 5; this.label='速度'; this.targetField = 'speedTarget';
    }
    accept(frame) {
      this.latest = frame; this.lastFrameAt = this.now();
      const r = this.run;
      if (!r) return;
      if (frame.fresh !== 7 || frame.mode === 0) { this.finish(`${this.label}环已停止`, false); return; }
      if (frame.mode === this.mode && frame[this.targetField]?.every((v, i) => Math.abs(v - r.targets[i]) < .00051)) {
        r.phase = 'tracking';
      } else if (r.phase === 'tracking' && frame.mode !== this.mode) this.finish(`设备已退出${this.label}环`, false);
    }
    start(targets) {
      if (!this.latest?.speedMeasured || this.now() - this.lastFrameAt > 500 || this.latest.fresh !== 7)
        throw new Error(`等待支持${this.label}环的 ${this.mode===6?'V8':'V4'} 固件遥测及三路有效反馈`);
      if (!this.run && this.latest.mode !== 1) throw new Error('请先恢复普通模式');
      if (targets.length !== 3 || targets.some(v => !Number.isFinite(v) || Math.abs(v) > 180))
        throw new Error(`三路目标${this.label}范围为 −180 到 180 ${this.mode===6?'°':'°/s'}`);
      this.run = {kind: 'speed', targets: targets.map(v => Math.round(v * 1000) / 1000),
        phase: 'arming', requestedAt: this.now(), lastSendAt: this.now()};
      this.sendTargets();
    }
    sendTargets() { this.send(`V,${this.run.targets.map(v => Math.round(v * 1000)).join(',')}`); }
    tick() {
      const r = this.run, now = this.now();
      if (!r) return;
      if (now - this.lastFrameAt > 500) { this.finish(`${this.label}遥测超时`); return; }
      if (r.phase === 'arming' && now - r.requestedAt > 1500) { this.finish(`设备未确认目标${this.label}`); return; }
      if (now - r.lastSendAt >= 50) { this.sendTargets(); r.lastSendAt = now; }
    }
    finish(reason = `停止${this.label}环`, stop = true) {
      if (!this.run) return;
      this.run = null;
      if (stop) this.send('S');
      this.onFinish(reason);
    }
  }
  class AngleLoop extends SpeedLoop {
    constructor(options) { super(options); this.mode=6; this.label='角度'; this.targetField='angleTarget'; }
    start(targets) {
      if (!this.latest?.anglePid) throw new Error('角度环需要V8固件遥测');
      super.start(targets); this.run.kind='position';
    }
    sendTargets() { this.send(`A,${this.run.targets.map(v=>Math.round(v*1000)).join(',')}`); }
  }
  class StaticFrictionGate {
    constructor(thresholdDeg = .05) {
      this.thresholdDeg = thresholdDeg; this.encoder = 0; this.startEncoder = 0;
      this.lastMotionMs = 0; this.direction = 0; this.starting = false;
    }
    restore(encoder, target) {
      this.encoder = encoder; this.startEncoder = encoder; this.direction = Math.sign(target);
      this.starting = this.direction !== 0;
    }
    update(encoder, target, now, enabled = true) {
      const countToDeg = 360 / (8192 * (3591 / 187));
      const moved = encoder !== this.encoder;
      if (moved) this.lastMotionMs = now;
      this.encoder = encoder;
      const direction = enabled ? Math.sign(target) : 0;
      if (direction !== this.direction) {
        this.starting = direction !== 0 && now - this.lastMotionMs >= 20;
        this.direction = direction; this.startEncoder = encoder;
      }
      if (Math.abs(encoder - this.startEncoder) * countToDeg >= this.thresholdDeg || direction === 0) this.starting = false;
      return this.starting ? direction : 0;
    }
  }
  function demoPidStep(state, params, target, feedback, dt) {
    const clamp = x => Math.max(-16384, Math.min(16384, x));
    const error = target - feedback;
    const integral = clamp(state.integral + params.ki * error * dt);
    const p = params.kp * error, d = state.initialized ? params.kd * (error - state.previousError) / dt : 0;
    const candidate = p + integral + d;
    if (!((candidate > 16384 && error > 0) || (candidate < -16384 && error < 0))) state.integral = integral;
    state.previousError = error; state.initialized = true;
    return clamp(p + state.integral + d);
  }
  return { INITIAL_ANGLE_DEG, RAW_PER_AMP, toAmp, toRaw, scaleRaw, parseTelemetry, LineReader, BreakawayTest, FixedCurrentTest, SpeedLoop, AngleLoop, demoPidStep, StaticFrictionGate };
});

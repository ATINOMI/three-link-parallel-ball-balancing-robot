(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MotorCore = api;
})(globalThis, function () {
  'use strict';
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

  return {parseTelemetry, LineReader};
});

(function (root, factory) {
  const api = factory(typeof module === 'object' && module.exports ? require('./core.js') : root.MotorCore);
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MotorDrawing = api;
})(globalThis, function (Core) {
  'use strict';
  const colors = ['#61ceae', '#e2b672', '#90b4eb'];
  function points(pose, angles) {
    const [x, y, z] = pose;
    const sx = Math.sin(x), cx = Math.cos(x), sy = Math.sin(y), cy = Math.cos(y);
    const yaw = Math.atan2(-sx * sy, cx + cy), sz = Math.sin(yaw), cz = Math.cos(yaw);
    const r = [[cy * cz, -cy * sz],
      [cx * sz + sx * sy * cz, cx * cz - sx * sy * sz],
      [sx * sz - cx * sy * cz, sx * cz + cx * sy * sz]];
    const p = [.0685 * (r[0][0] - r[1][1]), -.137 * r[1][0], z];
    const base = [], elbow = [], platform = [];
    for (let i = 0; i < 3; i++) {
      const alpha = i * 2 * Math.PI / 3, ex = Math.cos(alpha), ey = Math.sin(alpha);
      const t = angles[i] * Math.PI / 180;
      base.push([.126 * ex, .126 * ey, 0]);
      elbow.push([(.126 + .060 * Math.cos(t)) * ex,
        (.126 + .060 * Math.cos(t)) * ey, -.060 * Math.sin(t)]);
      platform.push(r.map((row, n) => p[n] + .137 * (row[0] * ex + row[1] * ey)));
    }
    return { base, elbow, platform, center: p };
  }
  function solve3(a, b) {
    const m = a.map((row, i) => [...row, b[i]]);
    for (let k = 0; k < 3; k++) {
      let pivot = k;
      for (let i = k + 1; i < 3; i++) if (Math.abs(m[i][k]) > Math.abs(m[pivot][k])) pivot = i;
      [m[k], m[pivot]] = [m[pivot], m[k]];
      for (let i = k + 1; i < 3; i++) {
        const ratio = m[i][k] / m[k][k];
        for (let j = k; j < 4; j++) m[i][j] -= ratio * m[k][j];
      }
    }
    const x = [0, 0, 0];
    for (let i = 2; i >= 0; i--) {
      let v = m[i][3];
      for (let j = i + 1; j < 3; j++) v -= m[i][j] * x[j];
      x[i] = v / m[i][i];
    }
    return x;
  }
  // 仅供离线演示：沿用机构长度闭环，让示意连杆仍满足几何关系。
  function demoPose(angles, seed) {
    const q = [...seed];
    const residual = pose => {
      const rig = points(pose, angles);
      return rig.platform.map((a, i) => a.reduce((sum, v, j) =>
        sum + (v - rig.elbow[i][j]) ** 2, 0) - .102 ** 2);
    };
    for (let step = 0; step < 5; step++) {
      const f = residual(q), jac = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for (let k = 0; k < 3; k++) {
        const plus = [...q], minus = [...q], eps = 1e-6;
        plus[k] += eps; minus[k] -= eps;
        const fp = residual(plus), fm = residual(minus);
        for (let i = 0; i < 3; i++) jac[i][k] = (fp[i] - fm[i]) / (2 * eps);
      }
      const delta = solve3(jac, f.map(v => -v));
      for (let k = 0; k < 3; k++) q[k] += delta[k];
    }
    return q;
  }
  function canvasContext(canvas) {
    const box = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, box.width), h = Math.max(1, box.height);
    const width = Math.round(w * dpr), height = Math.round(h * dpr);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const c = canvas.getContext('2d');
    c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
    return [c, w, h];
  }
  function mechanism(canvas, frame, view) {
    const [c, w, h] = canvasContext(canvas);
    const data = frame || { angles: Array(3).fill(Core.INITIAL_ANGLE_DEG), pose: [0, 0, .07699424] };
    const rig = points(data.pose, data.angles), scale = Math.min(w, h) * 2.25;
    const project = a => view === 'top' ? [w / 2 + a[0] * scale, h / 2 - a[1] * scale] :
      view === 'side' ? [w / 2 + a[0] * scale, h * .74 - a[2] * scale] :
      [w / 2 + (a[0] - a[1]) * .73 * scale,
        h * .68 + (a[0] + a[1]) * .29 * scale - a[2] * scale];
    function line(vertices, color, width, fill) {
      c.beginPath();
      vertices.forEach((a, i) => { const p = project(a); i ? c.lineTo(...p) : c.moveTo(...p); });
      if (fill) { c.closePath(); c.fillStyle = fill; c.fill(); }
      c.strokeStyle = color; c.lineWidth = width; c.stroke();
    }
    line(rig.base, '#52756b', 1, '#a6d5c411');
    line(rig.platform, '#a3dec4', 2, '#84cfb333');
    for (let i = 0; i < 3; i++) {
      line([rig.base[i], rig.elbow[i]], colors[i], 5);
      line([rig.elbow[i], rig.platform[i]], '#b8cec3', 3);
      for (const a of [rig.base[i], rig.elbow[i], rig.platform[i]]) {
        const p = project(a); c.beginPath(); c.arc(...p, 4, 0, Math.PI * 2);
        c.fillStyle = colors[i]; c.fill();
      }
      const p = project(rig.base[i]);
      c.fillStyle = colors[i]; c.font = 'bold 12px Consolas'; c.fillText(`M${i + 1}`, p[0] + 9, p[1] + 17);
    }
    const center = project(rig.center);
    c.fillStyle = '#e4f6ea'; c.beginPath(); c.arc(...center, 3, 0, Math.PI * 2); c.fill();
    if (!frame) { c.fillStyle = '#a9c3b8'; c.font = '12px Microsoft YaHei'; c.fillText('等待遥测 · 12.77° 标定示意', 20, h - 20); }
  }
  function appendPlotSample(history, frame, retentionMs = 60000) {
    let previous = history[history.length - 1];
    const delta = previous ? (frame.ms - previous.ms) >>> 0 : 0;
    if (previous && delta > 0x7fffffff) { history.length = 0; previous = null; }
    const t = previous ? previous.t + delta : frame.ms;
    history.push({...frame, t});
    while (history.length > 1 && history[0].t < t - retentionMs) history.shift();
    return history[history.length - 1];
  }
  function statistics(rows, value) {
    let count = 0, mean = 0, m2 = 0, sumSquares = 0, min = Infinity, max = -Infinity;
    for (const row of rows) {
      const v = value(row);
      if (!Number.isFinite(v)) continue;
      count++; const delta = v - mean; mean += delta / count; m2 += delta * (v - mean);
      sumSquares += v * v; min = Math.min(min, v); max = Math.max(max, v);
    }
    return count ? {count, min, max, mean, p2p: max - min, rms: Math.sqrt(sumSquares / count), std: Math.sqrt(m2 / count)} : null;
  }
  function visibleRows(history, options = {}) {
    if (!history.length) return [];
    const end = options.end ?? history[history.length - 1].t, duration = options.windowMs || 5000;
    return history.filter(row => row.t >= end - duration && row.t <= end);
  }
  function sampling(rows) {
    const gaps = [];
    for (let i = 1; i < rows.length; i++) { const gap = rows[i].t - rows[i - 1].t; if (gap > 0) gaps.push(gap); }
    if (!gaps.length) return null;
    gaps.sort((a, b) => a - b);
    const median = gaps[Math.floor(gaps.length / 2)];
    return {median, max: gaps[gaps.length - 1], hz: 1000 * gaps.length / gaps.reduce((a, b) => a + b, 0)};
  }
  function chartModel(history, series, reference = [], options = {}) {
    const end = options.end ?? history[history.length - 1]?.t ?? 0, windowMs = options.windowMs || 5000;
    const rows = visibleRows(history, {...options, end, windowMs});
    let lo = Infinity, hi = -Infinity;
    for (const row of rows) for (const s of series) { const v = s.value(row); if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }
    for (const ref of reference) if (Number.isFinite(ref.value)) { lo = Math.min(lo, ref.value); hi = Math.max(hi, ref.value); }
    if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
    const span = Math.max(hi - lo, options.minSpan || .002), center = (hi + lo) / 2;
    lo = center - span * .6; hi = center + span * .6;
    if (Number.isFinite(options.yMin) && Number.isFinite(options.yMax) && options.yMax > options.yMin) { lo = options.yMin; hi = options.yMax; }
    return {rows, start: end - windowMs, end, windowMs, lo, hi, sampling: sampling(rows)};
  }
  function nearestSample(rows, time) {
    let lo = 0, hi = rows.length - 1;
    while (lo < hi) { const mid = Math.floor((lo + hi) / 2); if (rows[mid].t < time) lo = mid + 1; else hi = mid; }
    if (!rows.length) return null;
    return lo > 0 && Math.abs(rows[lo - 1].t - time) <= Math.abs(rows[lo].t - time) ? rows[lo - 1] : rows[lo];
  }
  function trend(canvas, history, series, reference = [], options = {}) {
    const [c, w, h] = canvasContext(canvas), left = 88, right = Math.max(left + 1, w - 24), top = 24, bottom = Math.max(top + 1, h - 42);
    const model = chartModel(history, series, reference, options), {rows, lo, hi} = model;
    const x = t => left + (t - model.start) / model.windowMs * (right - left);
    const y = v => bottom - (v - lo) / (hi - lo) * (bottom - top);
    const digits = Math.min(6, Math.max(options.decimals || 0, 1 - Math.floor(Math.log10((hi - lo) / 6))));
    c.fillStyle = '#fbfdfb'; c.fillRect(left, top, right - left, bottom - top);
    c.font = '12px Consolas, monospace'; c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let i = 0; i <= 6; i++) {
      const value = lo + (hi - lo) * i / 6, py = y(value);
      c.strokeStyle = '#dce5e0'; c.lineWidth = 1; c.beginPath(); c.moveTo(left, py); c.lineTo(right, py); c.stroke();
      c.fillStyle = '#49655c'; c.fillText(value.toFixed(digits), left - 10, py);
    }
    c.textAlign = 'center';
    for (let i = 0; i <= 5; i++) {
      const t = model.start + model.windowMs * i / 5, px = x(t);
      c.strokeStyle = '#e5ece7'; c.beginPath(); c.moveTo(px, top); c.lineTo(px, bottom); c.stroke();
      c.fillStyle = '#49655c'; c.fillText(`${((t - model.end) / 1000).toFixed(model.windowMs <= 2000 ? 2 : 1)} s`, px, h - 17);
    }
    c.textAlign = 'left'; c.fillStyle = '#49655c'; c.fillText(options.unit || '', left, 10);
    if (!rows.length) { c.fillStyle = '#738579'; c.fillText('等待采样数据', left + 16, (top + bottom) / 2); return model; }
    c.save(); c.beginPath(); c.rect(left, top, right - left, bottom - top); c.clip();
    for (const ref of reference) {
      c.setLineDash([5, 4]); c.strokeStyle = ref.color; c.lineWidth = 1;
      c.beginPath(); c.moveTo(left, y(ref.value)); c.lineTo(right, y(ref.value)); c.stroke();
    }
    c.setLineDash([]);
    const gapLimit = Math.max(20, (model.sampling?.median || 20) * 3);
    for (const s of series) {
      c.strokeStyle = s.color; c.lineWidth = s.width || 1.7; c.beginPath();
      let previous = null;
      for (const row of rows) {
        const value = s.value(row);
        if (!Number.isFinite(value)) { previous = null; continue; }
        if (!previous || row.t - previous.t > gapLimit) c.moveTo(x(row.t), y(value)); else c.lineTo(x(row.t), y(value));
        previous = row;
      }
      c.stroke();
      if (options.points || rows.length <= 160) for (const row of rows) {
        const value = s.value(row); if (!Number.isFinite(value)) continue;
        c.beginPath(); c.arc(x(row.t), y(value), 1.7, 0, Math.PI * 2); c.fillStyle = s.color; c.fill();
      }
    }
    let selected = null;
    if (Number.isFinite(options.cursorRatio)) {
      const cursorX = Math.max(left, Math.min(right, options.cursorRatio * w));
      selected = nearestSample(rows, model.start + (cursorX - left) / (right - left) * model.windowMs);
      const px = x(selected.t);
      c.setLineDash([3, 3]); c.strokeStyle = '#49655c'; c.beginPath(); c.moveTo(px, top); c.lineTo(px, bottom); c.stroke(); c.setLineDash([]);
      for (const s of series) { const value = s.value(selected); if (!Number.isFinite(value)) continue; c.fillStyle = s.color; c.beginPath(); c.arc(px, y(value), 3.5, 0, Math.PI * 2); c.fill(); }
    }
    c.restore();
    if (selected) {
      const lines = [`设备 ${selected.ms} ms · 距窗口末端 ${((selected.t - model.end) / 1000).toFixed(3)} s`, ...series.map(s => `${s.label || '数值'}  ${s.value(selected).toFixed(options.decimals ?? 3)} ${options.unit || ''}`)];
      c.font = '13px Consolas, Microsoft YaHei';
      const boxW = Math.min(w - 20, Math.max(...lines.map(line => c.measureText(line).width)) + 24), boxH = lines.length * 22 + 12;
      const bx = Math.max(6, Math.min(w - boxW - 6, x(selected.t) + 15)), by = top + 8;
      c.fillStyle = '#ffffffee'; c.fillRect(bx, by, boxW, boxH); c.strokeStyle = '#9db5aa'; c.strokeRect(bx, by, boxW, boxH);
      c.fillStyle = '#183c35'; c.textAlign = 'left'; c.textBaseline = 'middle'; lines.forEach((line, i) => c.fillText(line, bx + 12, by + 17 + i * 22));
    }
    return model;
  }
  return { points, demoPose, mechanism, trend, appendPlotSample, statistics, visibleRows, sampling, chartModel, nearestSample };
});

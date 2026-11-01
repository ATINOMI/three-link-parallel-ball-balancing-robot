'use strict';
const { INITIAL_ANGLE_DEG, toAmp, toRaw, scaleRaw, BreakawayTest, FixedCurrentTest, SpeedLoop, AngleLoop, demoPidStep, StaticFrictionGate } = MotorCore;
const Tuning = MotorTuning;
const $ = id => document.getElementById(id);
const ui = {
  latest: null, lastFrameAt: -Infinity, history: [], results: [], view: 'iso',
  demo: null, connectedOnce: false, connecting: false, pendingParams: null,
  tuningDraft: null, tuningImported: false, applyingTuning: false, pendingTuning: null, rateInputsLoaded: false,
  lastVisualAt: -Infinity,
  charts: {speed: {paused: false, cursor: {}}, position: {paused: false, cursor: {}}, response: {paused: false, cursor: {}}},
  parametersLoaded: false, parametersLoadedScale: false, pendingScale: null, lastResult: null, legacy: false
};
const notify = text => { $('notice').textContent = text; };
const directionName = direction => direction < 0 ? '上升' : '下降';
const signed = (v, digits = 3) => Number.isFinite(v) ? `${v > 0 ? '+' : ''}${v.toFixed(digits)}` : '—';
const selectedIndex = () => Number($('motorSelect').value) - 1;
const selectedPidDirection = () => Number($('pidDirectionSelect').value) === 1 ? 1 : -1;

$('motorCards').innerHTML = [1, 2, 3].map((id, i) => `
  <article class="motor-card" data-motor="${i}">
    <h3>M${id} / 支链 ${id}</h3><div class="motor-angle"><span id="angle-${i}">—</span><small> °</small></div>
    <dl><dt>重力基线</dt><dd id="base-${i}">—</dd><dt>测试增量</dt><dd id="delta-${i}">—</dd>
    <dt>总指令</dt><dd id="total-${i}">—</dd><dt>电调反馈</dt><dd id="feedback-${i}">— raw</dd></dl>
  </article>`).join('');

const link = new MotorSerial.SerialLink({
  onFrame: receive,
  onStatus: () => {
    if (!link.connected && ui.connectedOnce) {
      finishActive('USB 已断开', false);
      notify('USB 已断开；固件心跳超时后，速度/角度闭环三路归零，其他电流测试撤回后回到普通补偿。');
    }
    renderStatus();
  },
  onError: error => { finishActive('USB 通信中断', false); notify(`USB 通信问题：${error.message}`); },
  onLegacy: () => {
    ui.legacy = true; notify('检测到旧版固件：请先烧录支持当前测试协议的新固件。'); renderStatus();
  }
});
const test = new BreakawayTest({ send: sendCommand, onFinish: recordResult });
const fixedTest = new FixedCurrentTest({ send: sendCommand, onFinish: recordResult });
const speedTest = new SpeedLoop({ send: sendCommand, onFinish: reason => {
  notify(`${reason}；速度环已结束，停止时请求三路零电流。`); renderStatus();
} });
const angleTest = new AngleLoop({send:sendCommand,onFinish:reason=>{notify(`${reason}；角度环已结束，停止时请求三路零电流。`);renderStatus();}});
const activeRun = () => test.run || fixedTest.run || speedTest.run || angleTest.run;
function finishActive(reason, restore = true) {
  test.finish(reason, restore); fixedTest.finish(reason, restore); speedTest.finish(reason, restore); angleTest.finish(reason, restore);
}

function sendCommand(line) {
  if (!ui.demo) return link.send(line);
  // 离线演示绝不调用串口写入。
  const d = ui.demo;
  if (line === 'G' || line === 'S') {
    d.mode = line === 'G' ? 1 : 0; d.motor = 0; d.increment = 0;
    d.targets = [0, 0, 0]; d.pidState = newDemoPidStates(); d.angleState = newDemoPidStates();d.angleSettleState=[0,0,0];
  } else if (line.startsWith('P,')) d.kg = Number(line.split(',')[1]) / 1000;
  else if (line.startsWith('R,')) d.outputScale = line.split(',').slice(1).map(x => Number(x) / 1000);
  else if (line.startsWith('H,')) {
    const [,tolerance,holdMs]=line.split(',').map(Number);d.angleSettling={toleranceDeg:tolerance/1000,holdMs};d.angleSettleState=[0,0,0];
  }
  else if (line.startsWith('A,')) {
    if(d.mode!==6){d.pidState=newDemoPidStates();d.angleState=newDemoPidStates();d.targets=[0,0,0];d.pidOutput=[0,0,0];}
    const target=line.split(',').slice(1).map(v=>Number(v)/1000);
    for(let i=0;i<3;i++)if(d.mode!==6||d.angleTargets[i]!==target[i])d.angleSettleState[i]=0;
    d.mode=6;d.motor=0;d.angleTargets=target;
  } else if (line.startsWith('J,')) {
    const [,motor,direction,kp,ki,kd,limit,minimum=0]=line.split(',').map(Number),i=motor-1,p=d.anglePid[i];
    const reset=d.angleDirection[i]===direction||p.maxSpeed!==limit/1000||p.minSpeed!==minimum/1000;
    p[direction<0?'up':'down']={kp:kp/1000,ki:ki/1000,kd:kd/1000};p.maxSpeed=limit/1000;p.minSpeed=minimum/1000;
    if(reset){d.angleSettleState[i]=0;d.angleState[i]={integral:0,previousError:0,initialized:false};if(d.mode===6){d.targets[i]=0;d.pidOutput[i]=0;}}
  }
  else if (line.startsWith('K,')) {
    const [, motor, direction, kp, ki, kd, friction] = line.split(',').map(Number), i = motor - 1;
    d.pidDirectional[i][direction < 0 ? 'up' : 'down'] = {kp: kp / 1000, ki: ki / 1000, kd: kd / 1000};
    d.pidSettings[i].friction = friction;
    if (d.pidDirection[i] === direction) {
      Object.assign(d.pidSettings[i], d.pidDirectional[i][direction < 0 ? 'up' : 'down']);
      d.pidState[i] = {integral: 0, previousError: 0, initialized: false}; d.pidOutput[i] = 0;
    }
  } else if (line.startsWith('U,')) {
    const [, speedPeriodMs, filterPeriodMs] = line.split(',').map(Number);
    d.timing = {speedPeriodMs, filterPeriodMs, cutoffHz: 20}; d.pidState = newDemoPidStates();
    d.speedDivider=0;d.filterDivider=0;d.pidOutput=[0,0,0];d.angleState=newDemoPidStates();d.angleSettleState=[0,0,0];if(d.mode===6)d.targets=[0,0,0];
    d.filterCoefficients=Tuning.designFilter(20,filterPeriodMs);
    d.filterStates=d.filteredSpeeds.map(v=>Tuning.resetFilter(d.filterCoefficients,v));
  } else if (line.startsWith('T,')) {
    const [, motor, mdeg] = line.split(',').map(Number);
    d.frictionGate[motor - 1].thresholdDeg = mdeg / 1000;
  } else if (line.startsWith('B,')) {
    const i = Number(line.split(',')[1]) - 1;
    if (d.mode >= 5 && d.targets[i] !== 0) {
      const encoder = Math.round((d.angles[i] - INITIAL_ANGLE_DEG) * (8192 * 3591 / 187 / 360));
      d.frictionGate[i].restore(encoder, d.targets[i]);
    }
  } else if (line.startsWith('V,')) {
    if (d.mode !== 5) { d.pidState = newDemoPidStates(); d.angleState=newDemoPidStates(); }
    d.mode = 5; d.motor = 0; d.targets = line.split(',').slice(1).map(v => Number(v) / 1000);
  } else if (line.startsWith('F,')) {
    const fields = line.split(','); d.mode = Number(fields[2]) ? 4 : 3; d.motor = Number(fields[1]); d.increment = Number(fields[3]);
  } else if (line.startsWith('D,')) {
    const fields = line.split(','); d.mode = 2; d.motor = Number(fields[1]); d.direction = Number(fields[2]); d.increment = Number(fields[3]);
  }
  return Promise.resolve(true);
}

function receive(frame) {
  ui.latest = frame; ui.lastFrameAt = performance.now(); ui.legacy = false;
  MotorDrawing.appendPlotSample(ui.history, frame);
  test.accept(frame); fixedTest.accept(frame); speedTest.accept(frame); angleTest.accept(frame);
  if (!ui.parametersLoaded) { if (!ui.tuningImported) $('kgInput').value = frame.kg; ui.parametersLoaded = true; }
  if (ui.pendingParams && Math.abs(frame.kg - ui.pendingParams.kg) < .00051) {
    ui.pendingParams = null; $('paramState').textContent = `已生效 · 设备 k_g = ${frame.kg.toFixed(3)}`;
  } else if (!ui.pendingParams) $('paramState').textContent = `设备 k_g = ${frame.kg.toFixed(3)} · 重力补偿按实测运动方向`;
  if (frame.outputScale) {
    if (!ui.parametersLoadedScale) {
      if (!ui.tuningImported) frame.outputScale.forEach((v, i) => { $(`outputScale${i + 1}`).value = v; });
      ui.parametersLoadedScale = true;
    }
    if (ui.pendingScale && frame.outputScale.every((v, i) => Math.abs(v - ui.pendingScale[i]) < .00051)) ui.pendingScale = null;
    $('outputScaleState').textContent = `${ui.pendingScale ? '等待设备确认 · ' : '设备比例 · '}${frame.outputScale.map((v, i) => `M${i + 1} ${v.toFixed(3)}`).join(' / ')}`;
  } else $('outputScaleState').textContent = '需要支持 V3 的新固件才能修改输出比例';
  if (ui.pendingTuning && Tuning.matchesFrame(ui.pendingTuning, frame)) {
    ui.pendingTuning = null; $('tuningFileState').textContent = '设备已回读确认全部调参值；速度/角度目标均未下发。';
  }
  refreshVisuals();
}

function renderReadings() {
  const f = ui.latest;
  if (!f) return;
  for (let i = 0; i < 3; i++) {
    $(`angle-${i}`).textContent = signed(f.angles[i], 3);
    for (const key of ['base', 'delta', 'total']) $(`${key}-${i}`).textContent = `${signed(toAmp(f[key][i]))} A`;
    $(`feedback-${i}`).textContent = `${f.feedback[i]} raw`;
  }
  $('poseRoll').textContent = (f.pose[0] * 180 / Math.PI).toFixed(2);
  $('posePitch').textContent = (f.pose[1] * 180 / Math.PI).toFixed(2);
  $('poseZ').textContent = (f.pose[2] * 1000).toFixed(2);
  const i = selectedIndex();
  $('currentBase').textContent = signed(f.base[i], 0);
  $('currentDelta').textContent = signed(f.delta[i], 0);
  $('currentTotal').textContent = signed(f.total[i], 0);
  $('currentDeltaLabel').textContent = f.mode >= 5 ? 'PID＋摩擦指令' : f.mode >= 3 && f.motor === i + 1 ? '固定给定指令' : '测试增量指令';
  document.querySelectorAll('.motor-card').forEach(card => card.classList.toggle('selected', Number(card.dataset.motor) === i));
  const r = activeRun();
  $('liveThreshold').textContent = r && (r.kind === 'fixed' || r.kind === 'speed' || r.kind === 'position') ? '—' : Number(r ? r.threshold : $('thresholdInput').value).toFixed(2);
  $('startAngle').textContent = r && r.startAngle !== undefined ? signed(r.startAngle, 2) : '—';
  $('displacement').textContent = r && r.startAngle !== undefined ? signed(f.angles[r.motor - 1] - r.startAngle, 2) : '—';
}

function renderStatus() {
  const fresh = ui.latest && performance.now() - ui.lastFrameAt <= 500;
  const active = Boolean(activeRun()) || ui.applyingTuning, attached = link.connected || Boolean(ui.demo);
  const stoppedLocally = ui.demo ? ui.demo.mode === 0 : link.emergencyLatched;
  $('linkState').textContent = ui.demo ? '离线演示' : link.connected ?
    (ui.legacy ? '旧固件协议' : link.failed ? 'USB 写入失败' : fresh ? 'USB 已连接' : '等待遥测') : '未连接';
  $('linkDetail').textContent = ui.demo ? '示意数据，不会发送 USB 指令' : fresh ?
    `${ui.latest.version || 'V3'} · ${ui.latest.fresh === 7 ? '三路反馈有效' : '电机反馈中断'}${['V10','V11'].includes(ui.latest.version) ? ' · 固定角度保护已取消' : ''}` : 'USB CDC · 数据未更新';
  $('linkDot').classList.toggle('online', Boolean(fresh && attached));
  $('linkDot').classList.toggle('stale', Boolean(link.connected && !fresh));
  $('deviceMode').textContent = !attached ? '等待设备' : !fresh ? '遥测未更新' :
    (stoppedLocally && ui.latest.mode !== 0 ? '急停指令已发出' :
      ['已急停 · 全零', '方向门控重力补偿', '重力 + 单电机增量', '固定电流指令', '固定给定 · 实测上升补偿', '三路速度 PID', '三路角度 / 速度串级 PID'][ui.latest.mode]);
  const ready = Boolean(attached && fresh && ui.latest.fresh === 7 && !link.failed);
  $('connectBtn').disabled = attached || ui.connecting;
  $('demoBtn').disabled = attached || ui.connecting;
  $('disconnectBtn').disabled = !attached;
  $('emergencyBtn').disabled = !attached;
  $('gravityBtn').disabled = !ready || active;
  $('upBtn').disabled = $('downBtn').disabled = !ready || active || stoppedLocally || ui.latest.mode !== 1;
  $('finishBtn').disabled = !activeRun();
  $('finishBtn').textContent = angleTest.run ? '结束角度环 · 三路零电流' : speedTest.run ? '结束速度环 · 三路零电流' : '结束测试 · 回到重力补偿';
  $('fixedApplyBtn').disabled = !ready || ui.applyingTuning || Boolean(test.run || speedTest.run || angleTest.run) || stoppedLocally || (!fixedTest.run && ui.latest.mode !== 1);
  $('fixedApplyBtn').textContent = fixedTest.run ? '更新固定指令' : '下发固定指令';
  $('fixedEndBtn').disabled = !fixedTest.run;
  $('fixedMotorSelect').disabled = active;
  $('fixedMotorSelect').value = $('motorSelect').value;
  $('fixedRawInput').disabled = $('fixedDurationInput').disabled = $('fixedGravityInput').disabled = Boolean(test.run || speedTest.run || angleTest.run);
  $('fixedState').textContent = fixedTest.run ?
    (fixedTest.run.phase === 'arming' ? '等待设备确认固定指令' : `M${fixedTest.run.motor} 给定 ${fixedTest.run.raw} raw · 重力前馈${fixedTest.run.gravityOn ? '按实测上升启用' : '关闭'}`) :
    '开关开启时，实测上升才补偿；下降和静止均不补偿。';
  for (const id of ['motorSelect', 'limitInput', 'rampInput', 'thresholdInput', 'durationInput']) $(id).disabled = active;
  $('kgInput').disabled = active;
  $('applyBtn').disabled = !ready || active;
  $('outputScaleApplyBtn').disabled = !ready || active || !ui.latest.outputScale;
  if (ui.applyingTuning && !activeRun()) $('phaseLabel').textContent = '正在下发配置，不发送运动指令';
  else if (active) {
    const r = activeRun();
    if (r.kind === 'position') {
      $('phaseLabel').textContent = r.phase === 'arming' ? '等待角度目标确认' : '三路角度 PID 跟踪中';
    } else if (r.kind === 'speed') {
      $('phaseLabel').textContent = r.phase === 'arming' ? '等待速度目标确认' : '三路速度 PID 跟踪中';
    } else if (r.kind === 'fixed') {
      const remaining = r.startedAt === undefined ? r.duration : Math.max(0, r.duration - (performance.now() - r.startedAt) / 1000);
      $('phaseLabel').textContent = r.phase === 'arming' ? `M${r.motor} · 等待固定指令确认` :
        `M${r.motor} 固定 ${r.raw} raw · 剩余 ${remaining.toFixed(1)} s`;
    } else {
      $('phaseLabel').textContent = r.phase === 'arming' ? `M${r.motor} ${directionName(r.direction)} · 重力按实测方向` :
        `M${r.motor} ${directionName(r.direction)} · 累计 ΔI ${signed(r.requestedDelta)} A`;
    }
  } else $('phaseLabel').textContent = ui.lastResult ? `${ui.lastResult.reason} · 测试指令已撤回` : '待测试';
  renderSpeedControls(ready, stoppedLocally); renderPositionControls(ready, stoppedLocally);
}

function begin(direction) {
  try {
    if (fixedTest.run || speedTest.run || angleTest.run) throw new Error('请先结束当前固定指令或速度模式');
    test.start(Number($('motorSelect').value), direction, {
      limit: Number($('limitInput').value), ramp: Number($('rampInput').value),
      threshold: Number($('thresholdInput').value), duration: Number($('durationInput').value)
    });
    notify('测试增量从零开始累加；三路重力补偿均按实测运动方向判断。');
  } catch (error) { notify(error.message); }
  renderStatus();
}

function recordResult(result) {
  ui.lastResult = result; ui.results.unshift(result); ui.results = ui.results.slice(0, 10);
  const success = result.reason === '检测到起动';
  const fixed = result.kind === 'fixed';
  $('resultCaption').textContent = fixed ? '固定模式最后实际下发指令' : '达到位移阈值时的测试增量';
  $('resultUnit').textContent = fixed ? 'raw' : 'A';
  $('resultDelta').textContent = fixed ? signed(toRaw(result.total), 0) : success ? signed(result.delta) : '—';
  $('resultHint').textContent = `M${result.motor} ${fixed ? '固定指令' : directionName(result.direction)} · ${result.reason}`;
  $('resultTotal').textContent = `重力 ${signed(result.baseline)} A ＋ 增量 ${signed(result.delta)} A ＝ 总指令 ${signed(result.total)} A。`;
  $('resultsBody').innerHTML = ui.results.map(r => `<tr>
    <td>M${r.motor} / ${r.kind === 'fixed' ? '固定' : directionName(r.direction)}</td><td>${r.reason}</td>
    <td>${signed(r.baseline)}</td><td>${signed(r.delta)}</td><td>${signed(r.total)}</td>
    <td>${signed(r.angle - r.startAngle, 2)}</td><td>${r.elapsed.toFixed(2)}</td></tr>`).join('');
  notify(`${result.reason}；${result.reason.includes('急停') ? '全部电流已请求清零' : '测试指令已撤回，回到原方向补偿逻辑'}。`);
  renderStatus();
}

function refreshVisuals(force = false) {
  const now = performance.now();
  if (!force && now - ui.lastVisualAt < 40) return;
  ui.lastVisualAt = now; renderReadings(); renderStatus(); draw();
}
function plotView(group) {
  const state = ui.charts[group], history = state.paused ? state.snapshot || [] : ui.history;
  const seconds = Number($(`${group}TimeWindow`).value), windowMs = (seconds > 0 ? seconds : 5) * 1000;
  const last = history[history.length - 1]?.t ?? 0, first = history[0]?.t ?? last;
  const slider = $(`${group}Offset`), maximum = Math.max(0, (last - first - windowMs) / 1000);
  slider.max = maximum.toFixed(3); slider.disabled = !state.paused || maximum <= 0;
  const offset = state.paused ? Math.max(0, Math.min(maximum, Number(slider.value) || 0)) : 0;
  slider.value = offset;
  $(`${group}OffsetText`).textContent = state.paused ? `已暂停 · 回看 ${offset.toFixed(3)} s` : '实时';
  $(`${group}PauseBtn`).textContent = state.paused ? '恢复实时图表' : '暂停图表';
  return {history, options: {windowMs, end: last - offset * 1000, points: $(`${group}Points`).checked}};
}
function axisOptions(prefix) {
  return $(`${prefix}FixedY`).checked ? {yMin: Number($(`${prefix}YMin`).value), yMax: Number($(`${prefix}YMax`).value)} : {};
}
function showSampling(id, rows) {
  const s = MotorDrawing.sampling(rows);
  $(id).textContent = s ? `${s.hz.toFixed(1)} Hz · 采样间隔中位 ${s.median.toFixed(1)} ms · 最大 ${s.max.toFixed(1)} ms · ${rows.length}点` : '等待至少两个设备采样点';
}
function drawResponseCharts() {
  const i = selectedIndex(), run = test.run, view = plotView('response');
  const reference = run && run.startAngle !== undefined ? [
    {value: run.startAngle, color: '#a6b5a6'}, {value: run.startAngle + run.direction * run.threshold, color: '#b57720'}
  ] : [];
  const common = view.options, cursor = ui.charts.response.cursor;
  MotorDrawing.trend($('angleChart'), view.history, [{label: `M${i + 1}角度`, value: f => f.angles[i], color: '#345e98'}], reference,
    {...common, ...axisOptions('angle'), unit: '°', decimals: 3, minSpan: .002, cursorRatio: cursor.angleChart});
  MotorDrawing.trend($('currentChart'), view.history, [
    {label: '重力分量', value: f => f.base[i], color: '#0c7467'},
    {label: 'PID/摩擦/测试分量', value: f => f.delta[i], color: '#b57720'},
    {label: '总电流指令', value: f => f.total[i], color: '#345e98'}
  ], [], {...common, ...axisOptions('current'), unit: 'raw', decimals: 0, minSpan: 2, cursorRatio: cursor.currentChart});
  const rows = MotorDrawing.visibleRows(view.history, common), angle = MotorDrawing.statistics(rows, f => f.angles[i]), current = MotorDrawing.statistics(rows, f => f.total[i]);
  $('angleStats').textContent = angle ? `范围 ${angle.min.toFixed(3)} ～ ${angle.max.toFixed(3)}° · 峰峰 ${angle.p2p.toFixed(3)}°` : '等待数据';
  $('currentStats').textContent = current ? `M${i + 1}总指令：${current.min} ～ ${current.max} raw · 峰峰 ${current.p2p} raw · 显示原始下发刻度` : '等待电流指令';
  showSampling('responseSampling', rows);
}
function draw() {
  MotorDrawing.mechanism($('mechanism'), ui.latest, ui.view);
  drawResponseCharts(); drawSpeedTracking(); drawPositionTracking();
}
function resetPlotViews() {
  for (const group of ['speed', 'position', 'response']) { ui.charts[group].paused = false; ui.charts[group].snapshot = null; ui.charts[group].cursor = {}; $(`${group}Offset`).value = 0; }
  ui.lastVisualAt = -Infinity;
}

function startDemo() {
  resetPlotViews();
  if (!ui.tuningImported) ui.tuningDraft = null;
  ui.rateInputsLoaded = false;ui.positionLoadedMotor=null;ui.positionTargetsLoaded=false;ui.positionSettlingLoaded=false;
  ui.demo = { mode: 1, motor: 0, increment: 0, kg: 1.4, timing: {speedPeriodMs: 5, filterPeriodMs: 5, cutoffHz: 20}, outputScale: [1, 1, 1],
    simMs: performance.now(), speedDivider:0, filterDivider:0, rawDivider:0, rawEncoders:[0,0,0], rawSpeeds:[0,0,0], filteredSpeeds:[0,0,0],
    filterCoefficients:Tuning.designFilter(20,5),filterStates:[0,1,2].map(()=>Tuning.resetFilter(Tuning.designFilter(20,5))),
    anglePid:[0,1,2].map(()=>({up:{kp:0,ki:0,kd:0},down:{kp:0,ki:0,kd:0},maxSpeed:50,minSpeed:0})),
    angleSettling:{toleranceDeg:.5,holdMs:20},angleSettleState:[0,0,0],angleSettleStarted:[0,0,0],
    angleTargets:[12.77,12.77,12.77],angleState:newDemoPidStates(),angleDirection:[-1,-1,-1],
    pidDirectional: [0,1,2].map(()=>({up:{kp:0,ki:0,kd:0},down:{kp:0,ki:0,kd:0}})), pidDirection: [-1,-1,-1],
    speeds: [0, 0, 0], targets: [0, 0, 0], pidSettings: [0, 1, 2].map(() => ({kp: 0, ki: 0, kd: 0, friction: 0})),
    frictionGate: [0, 1, 2].map(() => new StaticFrictionGate()),
    pidState: newDemoPidStates(), pidOutput: [0, 0, 0], frictionOutput: [0, 0, 0],
    angles: Array(3).fill(INITIAL_ANGLE_DEG), pose: [0, 0, .07699424], lastAt: performance.now() };
  ui.latest = null; ui.history = []; ui.parametersLoaded = false; ui.parametersLoadedScale = false; ui.pendingScale = null; ui.pidLoadedMotor = null; ui.pendingPid = null; link.failed = false;
  emitDemo(); notify('离线演示。起动阻力为示意设定，不代表真实装置参数。');
}
function emitDemo() {
  const d = ui.demo;
  if (!d) return;
  const now = performance.now(), dt = Math.min((now - d.lastAt) / 1000, .2); d.lastAt = now;
  const base = [], delta = [0, 0, 0], total = [];
  if (d.mode >= 5) {
    // 示意惯性模型，PID仍按5 ms计算；只用于验证界面与跟踪流程。
    for (let step = 0; step < Math.max(1, Math.round(dt / .001)); step++) {
      d.simMs++;const rawStep=++d.rawDivider>=5,filterStep=++d.filterDivider>=d.timing.filterPeriodMs,speedStep=++d.speedDivider>=d.timing.speedPeriodMs;
      if(rawStep)d.rawDivider=0;if(filterStep)d.filterDivider=0;if(speedStep)d.speedDivider=0;
      for (let i = 0; i < 3; i++) {
        if(rawStep){const count=Math.round((d.angles[i]-INITIAL_ANGLE_DEG)*(8192*3591/187/360));d.rawSpeeds[i]=(count-d.rawEncoders[i])*(360/(8192*3591/187))/.005;d.rawEncoders[i]=count;}
        if(filterStep)d.filteredSpeeds[i]=Tuning.filterStep(d.filterStates[i],d.filterCoefficients,d.rawSpeeds[i]);
        const gravity = toRaw(-.24 * d.kg * Math.cos(d.angles[i] * Math.PI / 180));
        if(d.mode===6){
          const error=d.angleTargets[i]-d.angles[i];
          if(Math.abs(error)<=d.angleSettling.toleranceDeg){
            if(d.angleSettleState[i]===0){d.angleSettleState[i]=1;d.angleSettleStarted[i]=d.simMs;}
            if(d.angleSettleState[i]===1&&d.simMs-d.angleSettleStarted[i]>=d.angleSettling.holdMs){d.angleSettleState[i]=2;d.pidOutput[i]=0;}
          }else d.angleSettleState[i]=0;
          if(d.angleSettleState[i]===2){d.targets[i]=0;d.angleState[i]={integral:0,previousError:0,initialized:false};}
          else if(speedStep){
            const direction=Math.sign(error)||d.angleDirection[i];
            if(direction!==d.angleDirection[i]){d.angleDirection[i]=direction;d.angleState[i]={integral:0,previousError:0,initialized:false};}
            const p=d.anglePid[i];d.targets[i]=positionPidStep(d.angleState[i],p[direction<0?'up':'down'],d.angleTargets[i],d.angles[i],d.timing.speedPeriodMs*.001,p.maxSpeed,d.angleSettleState[i]===0?(p.minSpeed??0):0);
          }
        }
        const pidDirection = d.targets[i] > 0 ? 1 : d.targets[i] < 0 ? -1 : d.pidDirection[i];
        if (pidDirection !== d.pidDirection[i]) {
          d.pidDirection[i] = pidDirection; Object.assign(d.pidSettings[i], d.pidDirectional[i][pidDirection < 0 ? 'up' : 'down']);
          d.pidState[i] = {integral:0,previousError:0,initialized:false}; d.pidOutput[i]=0;
        }
        if(speedStep)d.pidOutput[i] = scaleRaw(1, demoPidStep(d.pidState[i], d.pidSettings[i], d.targets[i], d.filteredSpeeds[i], d.timing.speedPeriodMs*.001));
        const encoder = Math.round((d.angles[i] - INITIAL_ANGLE_DEG) * (8192 * 3591 / 187 / 360));
        d.frictionOutput[i] = d.frictionGate[i].update(encoder, d.targets[i], d.simMs) * d.pidSettings[i].friction;
        const gravityCommand = d.speeds[i] < 0 ? gravity : 0;
        const command = Math.max(-16384, Math.min(16384, scaleRaw(gravityCommand + d.pidOutput[i] + d.frictionOutput[i], d.outputScale[i])));
        const drive = command - gravity;
        const resistance = Math.sign(d.speeds[i] || drive) * toRaw(.08);
        const acceleration = Math.abs(d.speeds[i]) < .01 && Math.abs(drive) < toRaw(.08) ? 0 : (drive - resistance) * .12 - 3 * d.speeds[i];
        d.speeds[i] += acceleration * .001; d.angles[i] += d.speeds[i] * .001;
        if (d.angles[i] <= -80) { d.mode = 0; d.targets = [0, 0, 0]; }
      }
      if (d.mode < 5) break;
    }
  }
  for (let i = 0; i < 3; i++) {
    let velocity = 0;
    if ((d.mode >= 2 && d.mode <= 4) && d.motor === i + 1) {
      const amp = toAmp(Math.max(-16384, Math.min(16384, scaleRaw(d.increment, d.outputScale[i]))));
      velocity = Math.sign(amp) * Math.max(0, Math.abs(amp) - .08) * 10;
      d.angles[i] = Math.max(-25, Math.min(INITIAL_ANGLE_DEG + 8, d.angles[i] + velocity * dt));
    }
    // 实测上升才补偿；固定模式关闭补偿开关时始终不补偿。
    const actualVelocity = d.mode >= 5 ? d.speeds[i] : velocity;
    const feedforwardOn = actualVelocity < 0 && !(d.mode === 3 && d.motor === i + 1);
    base[i] = d.mode !== 0 && feedforwardOn ? toRaw(-.24 * d.kg * Math.cos(d.angles[i] * Math.PI / 180)) : 0;
    if ((d.mode >= 2) && d.motor === i + 1) delta[i] = d.increment;
    if (d.mode >= 5) delta[i] = d.pidOutput[i] + d.frictionOutput[i];
    else {
      d.speeds[i] = velocity; d.filteredSpeeds[i]=velocity;d.rawSpeeds[i]=velocity;d.simMs=now;d.pidOutput[i] = 0; d.frictionOutput[i] = 0;
      const encoder = Math.round((d.angles[i] - INITIAL_ANGLE_DEG) * (8192 * 3591 / 187 / 360));
      d.frictionGate[i].update(encoder, 0, now, false);
    }
    total[i] = Math.max(-16384, Math.min(16384, scaleRaw(base[i] + delta[i], d.outputScale[i])));
    base[i] = Math.max(-16384, Math.min(16384, scaleRaw(base[i], d.outputScale[i])));
    delta[i] = total[i] - base[i];
  }
  d.pose = MotorDrawing.demoPose(d.angles, d.pose);
  receive({ speedMeasured: d.mode>=5?[...d.filteredSpeeds]:[...d.speeds], speedTarget: [...d.targets], pidSettings: d.pidSettings.map(v => ({...v})),
    pidOutput: [...d.pidOutput], frictionOutput: [...d.frictionOutput],
    pidDirectional: d.pidDirectional.map(p=>({up:{...p.up},down:{...p.down}})),
    angleTarget:[...d.angleTargets],anglePid:d.anglePid.map(p=>({up:{...p.up},down:{...p.down},maxSpeed:p.maxSpeed,...(p.minSpeed==null?{}:{minSpeed:p.minSpeed})})),
    angleSettling:{...d.angleSettling,state:d.mode===6?[...d.angleSettleState]:[0,0,0]},
    requestedRaw: d.mode >= 2 && d.mode <= 4 ? d.increment : 0, version: 'V11', timing: {...d.timing}, frictionMove: d.frictionGate.map(g => g.thresholdDeg), outputScale: [...d.outputScale], ms: Math.round(now), mode: d.mode, kg: d.kg, motor: d.motor, fresh: 7,
    angles: [...d.angles], pose: [...d.pose], base, delta, total, feedback: [...total] });
}

$('connectBtn').onclick = async () => {
  resetPlotViews(); if (!ui.tuningImported) ui.tuningDraft = null; ui.rateInputsLoaded = false;
  ui.positionLoadedMotor=null;ui.positionTargetsLoaded=false;ui.positionSettlingLoaded=false;ui.connecting = true; ui.parametersLoaded = false; ui.parametersLoadedScale = false; ui.pendingScale = null; ui.pendingParams = null; ui.pidLoadedMotor = null; ui.pendingPid = null;
  ui.latest = null; ui.lastFrameAt = -Infinity; ui.history = []; ui.legacy = false;
  renderStatus();
  try { await link.connect(); ui.connectedOnce = true; notify('USB 已连接，等待新固件遥测；连接动作不发送运动指令。'); }
  catch (error) { notify(`未连接：${error.message}`); }
  finally { ui.connecting = false; renderStatus(); }
};
$('demoBtn').onclick = startDemo;
$('disconnectBtn').onclick = async () => {
  const wasStopped = Boolean(speedTest.run || angleTest.run) || (ui.demo ? ui.demo.mode === 0 : link.emergencyLatched);
  finishActive('断开连接');
  if (ui.demo) { ui.demo = null; ui.latest = null; ui.history = []; }
  else await link.disconnect();
  notify(wasStopped ? '已断开，保持急停零输出。' : '已断开，测试增量撤回；设备继续按运动方向决定补偿。'); renderStatus(); draw();
};
function emergencyStop() {
  finishActive('手动急停', false);
  if (!ui.demo && !link.connected) {
    notify('设备未连接，无法发送急停指令。'); renderStatus(); return;
  }
  sendCommand('S');
  notify('已请求三路零电流。恢复时请明确点击“启用 / 恢复重力补偿”。'); renderStatus();
}
$('emergencyBtn').onclick = emergencyStop;
$('gravityBtn').onclick = () => { sendCommand('G'); notify('已恢复固件原方向补偿逻辑。'); };
$('applyBtn').onclick = () => {
  const kg = Number($('kgInput').value);
  if (!Number.isFinite(kg) || kg < 0 || kg > 5) { notify('补偿系数范围为 0 到 5'); return; }
  ui.pendingParams = { kg: Math.round(kg * 1000) / 1000, at: performance.now() };
  $('paramState').textContent = '等待设备确认'; sendCommand(`P,${Math.round(kg * 1000)}`);
};
$('outputScaleApplyBtn').onclick = () => {
  const values = [1, 2, 3].map(i => Number($(`outputScale${i}`).value));
  if (values.some(v => !Number.isFinite(v) || v < 0 || v > 5)) {
    notify('三路输出比例范围为 0 到 5'); return;
  }
  ui.pendingScale = values.map(v => Math.round(v * 1000) / 1000);
  $('outputScaleState').textContent = '等待设备确认';
  sendCommand(`R,${ui.pendingScale.map(v => Math.round(v * 1000)).join(',')}`);
};
$('fixedApplyBtn').onclick = () => {
  try {
    if (test.run || speedTest.run || angleTest.run) throw new Error('请先结束当前增量测试或速度模式');
    const text = $('fixedRawInput').value.trim();
    if (!text) throw new Error('请输入固定电流指令整数');
    const raw = Number(text), duration = Number($('fixedDurationInput').value);
    const gravityOn = $('fixedGravityInput').checked;
    if (fixedTest.run) fixedTest.update(raw, duration, gravityOn);
    else fixedTest.start(Number($('motorSelect').value), raw, duration, gravityOn);
    notify(`已请求固定给定 ${raw} raw，重力前馈${gravityOn ? '按实测上升启用' : '关闭'}。`);
  } catch (error) { notify(error.message); }
  renderStatus();
};
$('fixedGravityInput').onchange = () => {
  if (fixedTest.run) {
    const r = fixedTest.run;
    fixedTest.update(r.raw, r.duration, $('fixedGravityInput').checked);
    notify(`已更新按实测方向补偿的开关，固定给定仍为 ${r.raw} raw。`);
    renderStatus();
  }
};
$('fixedEndBtn').onclick = () => fixedTest.finish('手动结束固定指令');
$('upBtn').onclick = () => begin(-1);
$('downBtn').onclick = () => begin(1);
$('finishBtn').onclick = () => finishActive('手动结束');
$('motorSelect').onchange = () => { renderReadings(); draw(); };
$('thresholdInput').oninput = () => { $('liveThreshold').textContent = Number($('thresholdInput').value).toFixed(2); };
document.querySelectorAll('[data-view]').forEach(button => button.onclick = () => {
  ui.view = button.dataset.view;
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('active', b === button)); draw();
});
function newDemoPidStates() { return [0, 1, 2].map(() => ({integral: 0, previousError: 0, initialized: false})); }
function ensureTuningDraft() {
  if (!ui.tuningDraft) {
    try { ui.tuningDraft = Tuning.fromFrame(ui.latest); }
    catch {
      ui.tuningDraft = Tuning.defaults();
      if (ui.latest?.pidSettings) ui.latest.pidSettings.forEach((p,i)=>Object.assign(ui.tuningDraft.motors[i],{kp:p.kp,ki:p.ki,kd:p.kd,pidDown:{kp:p.kp,ki:p.ki,kd:p.kd},frictionRaw:p.friction,outputScale:ui.latest.outputScale?.[i]??1}));
    }
  }
  return ui.tuningDraft;
}
function stashPidDraft() {
  if (ui.pidLoadedMotor == null) return;
  const m = ensureTuningDraft().motors[ui.pidLoadedMotor];
  Object.assign(ui.pidLoadedDirection === 1 ? m.pidDown : m, {kp: Number($('pidKp').value), ki: Number($('pidKi').value), kd: Number($('pidKd').value)});
  Object.assign(m, {frictionRaw: Number($('pidFriction').value), frictionMoveDeg: Number($('frictionMoveInput').value)});
}
function loadPidDraft() {
  stashPidDraft();
  const i = Number($('pidMotorSelect').value || 1) - 1, m = ensureTuningDraft().motors[i];
  const p = selectedPidDirection() < 0 ? m : m.pidDown;
  $('pidKp').value = p.kp; $('pidKi').value = p.ki; $('pidKd').value = p.kd; $('pidFriction').value = m.frictionRaw;
  $('frictionMoveInput').value = m.frictionMoveDeg; ui.pidLoadedMotor = i; ui.pidLoadedDirection = selectedPidDirection();
}
function devicePid(motor, direction) {
  const gains = ui.latest?.pidDirectional?.[motor]?.[direction < 0 ? 'up' : 'down'];
  return gains ? {...gains, friction: ui.latest.pidSettings[motor].friction} : null;
}
function renderSpeedControls(ready, stoppedLocally) {
  const capable = Boolean(ui.latest?.speedMeasured), otherTest = Boolean(test.run || fixedTest.run || angleTest.run);
  const i = Number($('pidMotorSelect').value || 1) - 1;
  if (capable && ui.pidLoadedMotor == null) loadPidDraft();
  const p = devicePid(i, selectedPidDirection());
  if (ui.pendingPid) { const actual = devicePid(ui.pendingPid.motor, ui.pendingPid.direction); if (actual && ['kp','ki','kd','friction'].every(k=>Math.abs(actual[k]-ui.pendingPid[k])<.00051)) ui.pendingPid=null; }
  $('pidApplyBtn').disabled = !ready || !ui.latest?.pidDirectional || otherTest || ui.applyingTuning;
  $('frictionMoveApplyBtn').disabled = !ready || !ui.latest?.frictionMove || otherTest;
  $('frictionMoveState').textContent = ui.latest?.frictionMove ? `设备M${i + 1}撤销阈值 ${ui.latest.frictionMove[i].toFixed(3)}°` : '修改撤销阈值需要V5固件';
  $('frictionRestoreBtn').disabled = !ready || !capable || !(speedTest.run || angleTest.run) || ![5,6].includes(ui.latest?.mode) || !ui.latest?.speedTarget?.[i] || !ui.latest?.pidSettings?.[i]?.friction;
  $('frictionRestoreBtn').textContent = `恢复 M${i + 1} 静摩擦补偿`;
  $('speedStartBtn').disabled = !ready || !capable || otherTest || ui.applyingTuning || stoppedLocally || (!speedTest.run && ui.latest?.mode !== 1);
  $('speedStartBtn').textContent = speedTest.run ? '更新三路目标速度' : '启动速度环';
  $('speedStopBtn').disabled = !speedTest.run;
  $('pidParamState').textContent = p ? `${ui.pendingPid ? '等待参数确认 · ' : '设备参数 · '}M${i + 1} ${directionName(selectedPidDirection())} Kp ${p.kp.toFixed(3)} / Ki ${p.ki.toFixed(3)} / Kd ${p.kd.toFixed(3)} / 摩擦 ${p.friction} raw` : '上下独立PID需重新烧录V7固件';
  $('speedControlState').textContent = speedTest.run ? (speedTest.run.phase === 'arming' ? '等待设备确认目标速度' : '速度环运行中 · Q急停 / Esc停止') : '速度环未运行；下发参数不会启动电机';
  renderTuningTools(ready, otherTest);
  const chart = ui.charts.speed, snapshot = chart.snapshot || [];
  const frozenTime = (snapshot[snapshot.length - 1]?.t ?? 0) - (Number($('speedOffset').value) || 0) * 1000;
  const reading = chart.paused ? MotorDrawing.nearestSample(snapshot, frozenTime) : ui.latest;
  for (let m = 0; m < 3; m++) {
    const actual = reading?.speedMeasured?.[m], target = reading?.speedTarget?.[m];
    $(`speedReading${m + 1}`).textContent = Number.isFinite(actual) ? `${chart.paused ? "冻结" : "实时"} · 目标 ${signed(target, 3)} / 实测 ${signed(actual, 3)} / 误差 ${signed(target - actual, 3)}` : '等待速度遥测';
    $(`speedOutput${m + 1}`).textContent = reading?.pidOutput ? `PID原值 ${reading.pidOutput[m]} / 摩擦原值 ${reading.frictionOutput[m]} / 最终电流 ${reading.total[m]} raw` : 'PID — / 摩擦 — / 总电流 — raw';
  }
}
function drawSpeedTracking() {
  if ($('speedTrackWindow').hidden) return;
  const view = plotView('speed'), history = view.history.filter(f => f.speedMeasured);
  const options = {...view.options, ...axisOptions('speed'), unit: '°/s', decimals: 3, minSpan: .002};
  const selected = $('speedViewMotor').value === 'all' ? 'all' : Number($('speedViewMotor').value) || 1;
  const errorMode = $('speedPlotMode').value === 'error', summaries = [];
  $('speedLegend').innerHTML = errorMode ? '<span class="error-key">误差：实测−目标</span><span>零线＝目标</span><span>单位 °/s</span>' : '<span class="target-key">目标速度</span><span class="measured-key">实测速度</span><span>单位 °/s</span>';
  for (let i = 0; i < 3; i++) {
    const visible = selected === 'all' || selected === i + 1;
    $(`speedPlotBlock${i + 1}`).hidden = !visible;
    if (!visible) continue;
    const series = errorMode ? [{label: '实测−目标', value: f => f.speedMeasured[i] - f.speedTarget[i], color: '#a34d3e'}] : [
      {label: '目标速度', value: f => f.speedTarget[i], color: '#b57720', width: 1.4},
      {label: '实测速度', value: f => f.speedMeasured[i], color: '#345e98', width: 1.8}
    ];
    MotorDrawing.trend($(`speedChart${i + 1}`), history, series, errorMode ? [{value: 0, color: '#7c978a'}] : [],
      {...options, cursorRatio: ui.charts.speed.cursor[`speedChart${i + 1}`]});
    const rows = MotorDrawing.visibleRows(history, options), stats = MotorDrawing.statistics(rows, f => f.speedMeasured[i] - f.speedTarget[i]);
    if (stats) summaries.push(`M${i + 1}误差：峰峰 ${stats.p2p.toFixed(3)} / RMS ${stats.rms.toFixed(3)} / 标准差 ${stats.std.toFixed(3)} °/s`);
  }
  $('speedStats').textContent = summaries.join('　 |　 ') || '等待速度采样';
  showSampling('speedSampling', MotorDrawing.visibleRows(history, options));
}
function drawPositionTracking() {
  if ($('positionTrackWindow').hidden) return;
  const view = plotView('position'), history = view.history.filter(f => f.angleTarget);
  const options = {...view.options, ...axisOptions('position'), unit: '°', decimals: 3, minSpan: .002};
  const selected = $('positionViewMotor').value === 'all' ? 'all' : Number($('positionViewMotor').value) || 1;
  const errorMode = $('positionPlotMode').value === 'error', summaries = [];
  $('positionLegend').innerHTML = errorMode ? '<span class="error-key">误差：实测−目标</span><span>零线＝目标</span><span>单位 °</span>' : '<span class="target-key">目标角度</span><span class="measured-key">实测角度</span><span>单位 °</span>';
  for (let i = 0; i < 3; i++) {
    const visible = selected === 'all' || selected === i + 1;
    $(`positionPlotBlock${i + 1}`).hidden = !visible;
    if (!visible) continue;
    const series = errorMode ? [{label: '实测−目标', value: f => f.angles[i] - f.angleTarget[i], color: '#a34d3e'}] : [
      {label: '目标角度', value: f => f.angleTarget[i], color: '#b57720', width: 1.4},
      {label: '实测角度', value: f => f.angles[i], color: '#345e98', width: 1.8}
    ];
    MotorDrawing.trend($(`positionChart${i + 1}`), history, series, errorMode ? [{value: 0, color: '#7c978a'}] : [],
      {...options, cursorRatio: ui.charts.position.cursor[`positionChart${i + 1}`]});
    const rows = MotorDrawing.visibleRows(history, options), stats = MotorDrawing.statistics(rows, f => f.angles[i] - f.angleTarget[i]);
    if (stats) summaries.push(`M${i + 1}误差：峰峰 ${stats.p2p.toFixed(3)} / RMS ${stats.rms.toFixed(3)} / 标准差 ${stats.std.toFixed(3)} °`);
  }
  $('positionStats').textContent = summaries.join('　 |　 ') || '等待角度采样';
  showSampling('positionSampling', MotorDrawing.visibleRows(history, options));
}
for (const group of ['speed', 'position', 'response']) {
  $(`${group}PauseBtn`).onclick = () => {
    const state = ui.charts[group]; state.paused = !state.paused;
    state.snapshot = state.paused ? ui.history.slice() : null; $(`${group}Offset`).value = 0; draw();
  };
  $(`${group}TimeWindow`).onchange = draw; $(`${group}Points`).onchange = draw; $(`${group}Offset`).oninput = draw;
  const ids = group === 'response' ? ['angleChart','currentChart'] : [1,2,3].map(i=>`${group}Chart${i}`);
  for (const id of ids) {
    $(id).addEventListener('pointermove', event => {
      const rect = $(id).getBoundingClientRect(); ui.charts[group].cursor[id] = (event.clientX - rect.left) / rect.width;
      draw();
    });
    $(id).addEventListener('pointerleave', () => { delete ui.charts[group].cursor[id]; draw(); });
  }
}
for (const axis of ['speed', 'position', 'angle', 'current']) {
  $(`${axis}FixedY`).onchange = draw;
  $(`${axis}YMin`).oninput = $(`${axis}YMax`).oninput = draw;
}
$('speedViewMotor').onchange = $('speedPlotMode').onchange = draw;
$('positionViewMotor').onchange = $('positionPlotMode').onchange = draw;
for (const [button, panel, group] of [['speedExpandBtn', 'speedTrackWindow', 'speed'], ['responseExpandBtn', 'responsePanel', 'response'], ['positionExpandBtn','positionTrackWindow','position']]) {
  $(button).onclick = () => {
    const state = ui.charts[group]; state.expanded = !state.expanded;
    $(panel).classList.toggle('plot-maximized', state.expanded); $(button).textContent = state.expanded ? '还原图表' : '放大图表'; draw();
  };
}
if (typeof ResizeObserver === 'function') { const observer = new ResizeObserver(() => draw()); for (const id of ['fixedWindow', 'speedParamsWindow', 'speedTrackWindow', 'positionParamsWindow', 'positionTrackWindow', 'responsePanel']) observer.observe($(id)); }
$('pidMotorSelect').onchange = $('pidDirectionSelect').onchange = () => {loadPidDraft(); renderStatus();};
$('pidApplyBtn').onclick = () => {
  if (!ui.latest?.pidDirectional) {notify('上下独立PID需要重新烧录V7固件'); return;}
  const values = ['pidKp', 'pidKi', 'pidKd', 'pidFriction'].map(id => Number($(id).value));
  if (values.slice(0, 3).some(v => !Number.isFinite(v) || v < 0 || v > 100000) || !Number.isInteger(values[3]) || values[3] < 0 || values[3] > 16384) {
    notify('PID增益为0到100000，摩擦幅值为0到16384的整数'); return;
  }
  stashPidDraft();
  const encoded = values.slice(0, 3).map(v => Math.round(v * 1000));
  ui.pendingPid = {motor: Number($('pidMotorSelect').value || 1) - 1, direction: selectedPidDirection(), kp: encoded[0] / 1000, ki: encoded[1] / 1000, kd: encoded[2] / 1000, friction: values[3]};
  sendCommand(`K,${ui.pendingPid.motor + 1},${ui.pendingPid.direction},${encoded.join(',')},${values[3]}`);
  notify(`已下发${directionName(selectedPidDirection())}PID与该电机共用摩擦参数，等待回读；修改当前使用组时清除该路PID历史。`);
};
$('frictionMoveApplyBtn').onclick = () => {
  const value = Number($('frictionMoveInput').value);
  if (!Number.isFinite(value) || value < .001 || value > 1) { notify('撤销位移阈值范围为0.001到1°'); return; }
  const motor = Number($('pidMotorSelect').value || 1);
  stashPidDraft();
  sendCommand(`T,${motor},${Math.round(value * 1000)}`);
  notify(`已请求M${motor}撤销阈值 ${value.toFixed(3)}°；以设备回读值为准，修改不会重新开启补偿。`);
};
$('frictionRestoreBtn').onclick = () => {
  const motor = Number($('pidMotorSelect').value || 1);
  if (!(speedTest.run || angleTest.run) || ![5,6].includes(ui.latest?.mode) || !ui.latest.speedTarget?.[motor - 1]) return;
  sendCommand(`B,${motor}`);
  notify(`已请求恢复M${motor}静摩擦补偿；位移达到设备所设阈值后，在下一次1ms电流输出时撤掉。`);
};
$('speedStartBtn').onclick = () => {
  try {
    if (test.run || fixedTest.run || angleTest.run) throw new Error('请先结束当前电流测试或角度环');
    speedTest.start([1, 2, 3].map(i => Number($(`speedTarget${i}`).value)));
    $('speedTrackWindow').hidden = false; drawSpeedTracking();
    notify('已请求三路目标速度，单位°/s；负值抬升，正值下降。');
  } catch (error) { notify(error.message); }
  renderStatus();
};
$('speedStopBtn').onclick = () => speedTest.finish('手动停止速度环');

function readTestForm() {
  return {motor: Number($('motorSelect').value), limitA: Number($('limitInput').value), rampAps: Number($('rampInput').value), thresholdDeg: Number($('thresholdInput').value), durationS: Number($('durationInput').value), fixedRaw: Number($('fixedRawInput').value), fixedDurationS: Number($('fixedDurationInput').value), fixedGravity: Boolean($('fixedGravityInput').checked)};
}
function captureTuningForm() {
  stashPidDraft(); stashPositionDraft(); stashPositionSettling(); const c = JSON.parse(JSON.stringify(ensureTuningDraft()));
  c.kg = Number($('kgInput').value); c.source = 'draft';
  c.motors.forEach((m,i) => {m.outputScale = Number($(`outputScale${i+1}`).value);});
  c.speedTargets = [1,2,3].map(i => Number($(`speedTarget${i}`).value));
  if(c.position)c.position.targets=[1,2,3].map(i=>Number($(`positionTarget${i}`).value));
  c.timing = {speedPeriodMs: Tuning.periodFromHz(Number($('speedRateInput').value)), filterPeriodMs: Tuning.periodFromHz(Number($('filterRateInput').value), true), cutoffHz: ui.latest?.timing?.cutoffHz || 20};
  c.test = readTestForm(); c.createdAt = new Date().toISOString(); return Tuning.normalize(c);
}
function buildTuningExport(source = $('tuningExportSource').value) {
  const c = source === 'device' ? Tuning.fromFrame(ui.latest) : captureTuningForm();
  c.createdAt = new Date().toISOString(); return Tuning.normalize(c);
}
function loadTuningFile(value) {
  const c = Tuning.normalize(value); if (activeRun()) throw new Error('请先结束当前测试/速度环再导入文件'); ui.tuningDraft = c; ui.tuningImported = true; ui.pidLoadedMotor = null;
  $('kgInput').value = c.kg; c.motors.forEach((m,i) => {$(`outputScale${i+1}`).value = m.outputScale;});
  c.speedTargets.forEach((v,i) => {$(`speedTarget${i+1}`).value = v;});
  if (c.timing) {$('speedRateInput').value = 1000/c.timing.speedPeriodMs; $('filterRateInput').value = 1000/c.timing.filterPeriodMs; ui.rateInputsLoaded = true;}
  if (c.test) {
    const t=c.test; $('motorSelect').value=t.motor; $('fixedMotorSelect').value=t.motor;
    $('limitInput').value=t.limitA; $('rampInput').value=t.rampAps; $('thresholdInput').value=t.thresholdDeg; $('durationInput').value=t.durationS;
    $('fixedRawInput').value=t.fixedRaw; $('fixedDurationInput').value=t.fixedDurationS; $('fixedGravityInput').checked=t.fixedGravity;
  }
  $('pidMotorSelect').value='1'; $('pidDirectionSelect').value='-1'; loadPidDraft();
  ui.positionLoadedMotor=null; ui.positionTargetsLoaded=false;ui.positionSettlingLoaded=false;
  if(c.position){c.position.targets.forEach((v,i)=>{$(`positionTarget${i+1}`).value=v;});ui.positionTargetsLoaded=true;loadPositionDraft();}
  $('tuningExportSource').value='draft'; $('tuningFileState').textContent='已导入三路参数到表单，尚未下发；目标速度也只填表。';
  notify('文件已载入。检查参数后可应用全部参数；启动速度环仍需单独操作。'); renderStatus(); return c;
}
function renderTuningTools(ready, otherTest) {
  const t=ui.latest?.timing;
  if (!ui.rateInputsLoaded && t && (!ui.tuningImported || !ui.tuningDraft?.timing)) {$('speedRateInput').value=1000/t.speedPeriodMs; $('filterRateInput').value=1000/t.filterPeriodMs; ui.rateInputsLoaded=true;}
  if (!ui.rateInputsLoaded && ui.demo) {$('speedRateInput').value=200; $('filterRateInput').value=200; ui.rateInputsLoaded=true;}
  $('timingApplyBtn').disabled=!ready || !t || otherTest || ui.applyingTuning;
  $('tuningImportBtn').disabled=Boolean(activeRun()) || ui.applyingTuning;
  $('tuningApplyAllBtn').disabled=!ready || !ui.latest?.pidDirectional || Boolean(activeRun()) || ui.applyingTuning;
  $('tuningExportBtn').disabled=$('tuningExportSource').value==='device' && !ui.latest?.frictionMove;
  $('timingState').textContent=t ? `设备：速度环 ${(1000/t.speedPeriodMs).toFixed(2)}Hz (${t.speedPeriodMs}ms) / 滤波 ${(1000/t.filterPeriodMs).toFixed(2)}Hz (${t.filterPeriodMs}ms) / 截止 ${t.cutoffHz}Hz` : '频率控制需重新烧录V6固件；旧固件仍可导出已生效参数';
  const phase=t ? [1,2,5,10,20].map(f=>Tuning.response(t.cutoffHz,t.filterPeriodMs,f)).filter(Boolean) : [];
  $('filterPhaseState').textContent=phase.length ? `当前数字滤波本身滞后：${phase.map(r=>`${r.frequency}Hz ${(-r.phase).toFixed(1)}°`).join(' / ')}。不等于速度环带宽或相位裕度。` : '二阶巴特沃斯截止20Hz；等待设备回读实际更新频率';
}
$('tuningExportSource').onchange=renderStatus;
$('tuningExportBtn').onclick=()=>{
  try {
    const c=buildTuningExport(), blob=new Blob([JSON.stringify(c,null,2)],{type:'application/json'}), url=URL.createObjectURL(blob);
    const a=document.createElement('a'); a.href=url; a.download=`3motor-tuning-${c.createdAt.replace(/[:.]/g,'-')}.json`; a.click(); URL.revokeObjectURL(url);
    $('tuningFileState').textContent=`已导出${c.source==='device'?'设备已生效参数':'当前表单（含未应用值）'}，包含三台电机。`;
  } catch(error) {notify(error.message);}
};
$('tuningImportBtn').onclick=()=>{$('tuningFileInput').value='';$('tuningFileInput').click();};
$('tuningFileInput').onchange=async()=>{
  const file=$('tuningFileInput').files?.[0]; if(!file)return;
  try {loadTuningFile(JSON.parse(await file.text()));} catch(error){$('tuningFileState').textContent=`导入失败：${error.message}`;notify(error.message);}
};
$('tuningApplyAllBtn').onclick=async()=>{
  try {
    if(activeRun())throw new Error('先结束当前测试/速度环，再应用全部参数');
    const c=captureTuningForm();
    if(!ui.latest?.pidDirectional)throw new Error('应用上下独立PID参数需要V7固件');
    if(c.position&&!ui.latest.anglePid)throw new Error('文件包含角度配置，需要V8固件');
    if(c.position?.settling&&!ui.latest.angleSettling)throw new Error('文件包含稳定到位配置，需要V9固件');
    if(c.position?.pid.some(p=>p.minSpeed!=null)&&ui.latest.anglePid[0].minSpeed==null)throw new Error('文件包含最小角速度，需要V11固件');
    ui.applyingTuning=true; renderStatus();
    for(const command of Tuning.commands(c)){const ok=await sendCommand(command);if(ok===false)throw new Error('参数下发失败，请检查USB');}
    ui.pendingTuning=c; $('tuningFileState').textContent='全部配置已下发，等待设备回读；未发送目标速度或电流测试指令。';
  } catch(error){notify(error.message);} finally {ui.applyingTuning=false;renderStatus();}
};
$('timingApplyBtn').onclick=()=>{
  try {
    const speed=Tuning.periodFromHz(Number($('speedRateInput').value)),filter=Tuning.periodFromHz(Number($('filterRateInput').value),true);
    sendCommand(`U,${speed},${filter}`);
    notify(`已请求速度环 ${1000/speed}Hz、滤波 ${1000/filter}Hz；将复位PID历史，实际值以设备回读为准。`);
  } catch(error){notify(error.message);}
};
for(const id of ['pidKp','pidKi','pidKd','pidFriction','frictionMoveInput'])$(id).oninput=stashPidDraft;

function attachSpeedWindow(panelId, handleId, openId, closeId, onClose, canDrag = () => true) {
  const panel = $(panelId), handle = $(handleId);
  let drag = null;
  function position(x, y) {
    const rect = panel.getBoundingClientRect(), title = handle.getBoundingClientRect();
    const minY = Math.max(8, (document.querySelector?.('.topbar')?.getBoundingClientRect().bottom || 0) + 8);
    Object.assign(panel.style, {left: `${Math.max(Math.min(12, 100 - rect.width), Math.min(window.innerWidth - 100, x))}px`,
      top: `${Math.max(minY, Math.min(Math.max(minY, window.innerHeight - title.height - 8), y))}px`, right: 'auto', bottom: 'auto'});
  }
  $(openId).onclick = () => { panel.hidden = false; drawSpeedTracking(); };
  $(closeId).onclick = () => { if (onClose) onClose(); panel.hidden = true; $(openId).focus({preventScroll: true}); };
  function begin(event) {
    if (!canDrag() || event.button !== undefined && event.button !== 0 || event.target?.closest?.('button')) return;
    event.preventDefault(); if (drag) return;
    const rect = panel.getBoundingClientRect();
    drag = {id: event.pointerId, x: event.clientX, y: event.clientY, left: rect.left, top: rect.top};
    handle.classList.add('dragging');
  }
  function move(event) {
    if (!drag || event.pointerId !== undefined && drag.id !== undefined && event.pointerId !== drag.id) return;
    position(drag.left + event.clientX - drag.x, drag.top + event.clientY - drag.y);
  }
  function end(event) {
    if (drag && event?.pointerId !== undefined && drag.id !== undefined && event.pointerId !== drag.id) return;
    drag = null; handle.classList.remove('dragging');
  }
  handle.addEventListener('pointerdown', begin); handle.addEventListener('mousedown', begin);
  for (const type of ['pointermove', 'mousemove']) document.addEventListener(type, move, true);
  for (const type of ['pointerup', 'pointercancel', 'mouseup']) document.addEventListener(type, end, true);
  window.addEventListener('blur', end);
  handle.addEventListener('keydown', event => {
    if (event.target !== handle || !canDrag()) return;
    const delta = {ArrowLeft: [-16, 0], ArrowRight: [16, 0], ArrowUp: [0, -16], ArrowDown: [0, 16]}[event.key];
    if (!delta) return;
    event.preventDefault(); const rect = panel.getBoundingClientRect(); position(rect.left + delta[0], rect.top + delta[1]);
  });
  window.addEventListener('resize', () => { if (!panel.hidden && panel.style.left) { const rect = panel.getBoundingClientRect(); position(rect.left, rect.top); } });
}
attachSpeedWindow('speedParamsWindow', 'speedParamsHandle', 'speedParamsOpenBtn', 'speedParamsCloseBtn', () => speedTest.finish('关闭速度控制浮窗'));
attachSpeedWindow('speedTrackWindow', 'speedTrackHandle', 'speedTrackOpenBtn', 'speedTrackCloseBtn');

// 同页非模态浮窗，不创建新浏览器窗口，也不转移串口连接。
const fixedWindow = $('fixedWindow');
const fixedHandle = $('fixedWindowHandle');
let fixedDrag = null;
function positionFixedWindow(x, y) {
  const rect = fixedWindow.getBoundingClientRect();
  // 只要求标题栏可见，窗体比视口大时仍能拖动。
  const titleHeight = fixedHandle.getBoundingClientRect().height;
  const minX = Math.min(12, 100 - rect.width);
  const maxX = Math.max(12, window.innerWidth - 100);
  const minY = Math.max(8, (document.querySelector?.('.topbar')?.getBoundingClientRect().bottom || 0) + 8);
  const maxY = Math.max(minY, window.innerHeight - titleHeight - 8);
  Object.assign(fixedWindow.style, {
    left: `${Math.min(maxX, Math.max(minX, x))}px`,
    top: `${Math.min(maxY, Math.max(minY, y))}px`, right: 'auto', bottom: 'auto'
  });
}
$('fixedOpenBtn').onclick = () => {
  fixedWindow.hidden = false; $('fixedOpenBtn').hidden = true;
  if (fixedWindow.style.left) {
    const rect = fixedWindow.getBoundingClientRect(); positionFixedWindow(rect.left, rect.top);
  }
  $('fixedRawInput').focus({ preventScroll: true });
};
$('fixedCloseBtn').onclick = () => {
  fixedTest.finish('关闭固定指令浮窗');
  fixedWindow.hidden = true; $('fixedOpenBtn').hidden = false;
  $('fixedOpenBtn').focus({ preventScroll: true });
};
$('fixedMotorSelect').onchange = () => {
  $('motorSelect').value = $('fixedMotorSelect').value;
  renderReadings(); draw();
};
function beginFixedDrag(event) {
  if ((event.button !== undefined && event.button !== 0) || event.target?.closest?.('button')) return;
  event.preventDefault();
  if (fixedDrag) return;
  const rect = fixedWindow.getBoundingClientRect();
  fixedDrag = { id: event.pointerId, x: event.clientX, y: event.clientY, left: rect.left, top: rect.top };
  fixedHandle.classList.add('dragging');
}
function moveFixedDrag(event) {
  if (!fixedDrag) return;
  if (event.pointerId !== undefined && fixedDrag.id !== undefined && event.pointerId !== fixedDrag.id) return;
  positionFixedWindow(fixedDrag.left + event.clientX - fixedDrag.x, fixedDrag.top + event.clientY - fixedDrag.y);
}
function endFixedDrag(event) {
  if (fixedDrag && event?.pointerId !== undefined && fixedDrag.id !== undefined && event.pointerId !== fixedDrag.id) return;
  fixedDrag = null; fixedHandle.classList.remove('dragging');
}
// 文档级监听：离开标题栏仍继续拖，不依赖嵌入式浏览器的指针捕获。
fixedHandle.addEventListener('pointerdown', beginFixedDrag);
fixedHandle.addEventListener('mousedown', beginFixedDrag);
fixedHandle.addEventListener('dragstart', event => event.preventDefault());
for (const type of ['pointermove', 'mousemove']) document.addEventListener(type, moveFixedDrag, true);
for (const type of ['pointerup', 'pointercancel', 'mouseup']) document.addEventListener(type, endFixedDrag, true);
window.addEventListener('blur', endFixedDrag);
fixedHandle.addEventListener('keydown', event => {
  if (event.target !== fixedHandle) return;
  const step = { ArrowLeft: [-16, 0], ArrowRight: [16, 0], ArrowUp: [0, -16], ArrowDown: [0, 16] }[event.key];
  if (!step) return;
  event.preventDefault(); const rect = fixedWindow.getBoundingClientRect();
  positionFixedWindow(rect.left + step[0], rect.top + step[1]);
});
window.addEventListener('resize', () => {
  if (!fixedWindow.hidden && fixedWindow.style.left) {
    const rect = fixedWindow.getBoundingClientRect(); positionFixedWindow(rect.left, rect.top);
  }
});
window.addEventListener('blur', () => finishActive('窗口失焦'));
document.addEventListener('visibilitychange', () => { if (document.hidden) finishActive('页面隐藏'); });
window.addEventListener('beforeunload', () => finishActive('关闭页面'));
window.addEventListener('keydown', event => {
  if (event.code === 'KeyQ' || event.key === 'q' || event.key === 'Q') {
    event.preventDefault(); event.stopPropagation();
    if (!event.repeat) emergencyStop();
    return;
  }
  if (event.key === 'Escape') finishActive('手动结束');
}, true); // 捕获阶段：输入框、浮窗中有焦点时也能急停。
window.addEventListener('resize', draw);
setInterval(() => {
  test.tick(); fixedTest.tick(); speedTest.tick(); angleTest.tick(); emitDemo(); refreshVisuals();
  if (ui.pendingParams && performance.now() - ui.pendingParams.at > 1500) {
    $('paramState').textContent = '设备尚未确认参数，请检查 USB / 固件版本'; ui.pendingParams = null;
  }
}, 50);

function defaultPositionDraft(){return {targets:[12.77,12.77,12.77],pid:[0,1,2].map(()=>({up:{kp:0,ki:0,kd:0},down:{kp:0,ki:0,kd:0},maxSpeed:50}))};}
function ensurePositionDraft(){
  const c=ensureTuningDraft();
  if(!c.position)c.position=ui.latest?.anglePid?{targets:[...ui.latest.angleTarget],pid:ui.latest.anglePid.map(p=>({up:{...p.up},down:{...p.down},maxSpeed:p.maxSpeed,...(p.minSpeed==null?{}:{minSpeed:p.minSpeed})}))}:defaultPositionDraft();
  return c.position;
}
function stashPositionDraft(){
  if(ui.positionLoadedMotor==null)return;
  const p=ensurePositionDraft().pid[ui.positionLoadedMotor];
  p[ui.positionLoadedDirection<0?'up':'down']={kp:Number($('positionKp').value),ki:Number($('positionKi').value),kd:Number($('positionKd').value)};
  p.maxSpeed=Number($('positionMaxSpeed').value);
  if(ui.latest?.anglePid?.[ui.positionLoadedMotor]?.minSpeed!=null||p.minSpeed!=null)
    p.minSpeed=Number($('positionMinSpeed').value);
}
function loadPositionDraft(){
  stashPositionDraft();const i=Number($('positionMotorSelect').value||1)-1,d=Number($('positionDirectionSelect').value)===1?1:-1,p=ensurePositionDraft().pid[i],g=p[d<0?'up':'down'];
  $('positionKp').value=g.kp;$('positionKi').value=g.ki;$('positionKd').value=g.kd;$('positionMaxSpeed').value=p.maxSpeed;$('positionMinSpeed').value=p.minSpeed??0;
  ui.positionLoadedMotor=i;ui.positionLoadedDirection=d;
}
function renderPositionControls(ready,stoppedLocally){
  const capable=Boolean(ui.latest?.anglePid),other=Boolean(test.run||fixedTest.run||speedTest.run),i=Number($('positionMotorSelect').value||1)-1,d=Number($('positionDirectionSelect').value)===1?1:-1;
  if(capable&&ui.positionLoadedMotor==null)loadPositionDraft();
  if(capable&&!ui.positionTargetsLoaded){const targets=ensurePositionDraft().targets;targets.forEach((v,i)=>{$(`positionTarget${i+1}`).value=v;});ui.positionTargetsLoaded=true;}
  const p=ui.latest?.anglePid?.[i],g=p?.[d<0?'up':'down'];
  if(ui.pendingPosition){const pending=ui.pendingPosition,actual=ui.latest?.anglePid?.[pending.motor],gain=actual?.[pending.direction<0?'up':'down'];if(gain&&['kp','ki','kd'].every(k=>Math.abs(gain[k]-pending[k])<.00051)&&Math.abs(actual.maxSpeed-pending.maxSpeed)<.00051&&(pending.minSpeed==null||Math.abs(actual.minSpeed-pending.minSpeed)<.00051))ui.pendingPosition=null;}
  $('positionMinSpeed').disabled=p?.minSpeed==null;
  $('positionApplyBtn').disabled=!ready||!capable||other||ui.applyingTuning;
  $('positionStartBtn').disabled=!ready||!capable||other||ui.applyingTuning||stoppedLocally||(!angleTest.run&&ui.latest?.mode!==1);
  $('positionStopBtn').disabled=!angleTest.run;
  $('positionStartBtn').textContent=angleTest.run?'更新三路目标角度':'启动角度环';
  $('positionParamState').textContent=g?`${ui.pendingPosition?'等待确认 · ': '设备参数 · '}M${i+1} ${directionName(d)} Kp ${g.kp.toFixed(3)} / Ki ${g.ki.toFixed(3)} / Kd ${g.kd.toFixed(3)} / 上限 ${p.maxSpeed.toFixed(3)} / 最小 ${p.minSpeed==null?'需V11':p.minSpeed.toFixed(3)} °/s`:'角度环需要重新烧录V8固件';
  $('positionTimingState').textContent=ui.latest?.timing?`角度环与速度环同步更新：${(1000/ui.latest.timing.speedPeriodMs).toFixed(2)} Hz；频率在速度调参浮窗修改`:'角度环与速度环同步更新';
  $('positionControlState').textContent=angleTest.run?(angleTest.run.phase==='arming'?'等待设备确认角度目标':'角度串级闭环运行中 · Q急停 / Esc停止'):'角度环未运行；修改参数不会启动电机';
  renderPositionSettling(ready,other);
  const state=ui.charts.position,last=state.snapshot?.at(-1)?.t??0,reading=state.paused?MotorDrawing.nearestSample(state.snapshot||[],last-(Number($('positionOffset').value)||0)*1000):ui.latest;
  for(let m=0;m<3;m++){
    const a=reading?.angles?.[m],target=reading?.angleTarget?.[m];
    $(`positionReading${m+1}`).textContent=Number.isFinite(target)?`${state.paused?'冻结':'实时'} · 目标 ${signed(target)} / 实测 ${signed(a)} / 误差 ${signed(target-a)}`:'等待角度遥测';
    $(`positionOutput${m+1}`).textContent=reading?.anglePid?`${positionArrivalText(reading,m)} · 外环目标速度 ${signed(reading.speedTarget[m])} °/s / PID电流 ${reading.pidOutput[m]} / 最终电流 ${reading.total[m]} raw`:'等待串级控制数据';
  }
}
function positionPidStep(state,p,target,feedback,dt,limit,minimum=0){
  const clamp=v=>Math.max(-limit,Math.min(limit,v)),error=target-feedback,integral=clamp(state.integral+p.ki*error*dt);
  const proportional=p.kp*error,derivative=state.initialized?p.kd*(error-state.previousError)/dt:0,candidate=proportional+integral+derivative;
  if(!((candidate>limit&&error>0)||(candidate< -limit&&error<0)))state.integral=integral;
  state.previousError=error;state.initialized=true;const output=clamp(proportional+state.integral+derivative);
  return output>0&&output<minimum?minimum:output<0&&output> -minimum?-minimum:output;
}

function positionArrivalText(frame,i){
  if(!frame?.angleSettling)return '到位状态需V9固件';
  if(frame.mode!==6)return '未运行';
  return ['纠偏中','范围内 · 稳定确认中','成功到位 · 暂停角度纠偏'][frame.angleSettling.state[i]];
}
function stashPositionSettling(){
  if(!ui.positionSettlingLoaded)return;
  ensurePositionDraft().settling={toleranceDeg:Number($('positionTolerance').value),holdMs:Number($('positionHoldMs').value)};
}
function renderPositionSettling(ready,other){
  const p=ui.latest?.angleSettling;
  if(p&&!ui.positionSettlingLoaded){
    const config=ensurePositionDraft().settling||p;
    $('positionTolerance').value=config.toleranceDeg;$('positionHoldMs').value=config.holdMs;ui.positionSettlingLoaded=true;
  }
  if(ui.pendingSettling&&p&&Math.abs(p.toleranceDeg-ui.pendingSettling.toleranceDeg)<.00051&&p.holdMs===ui.pendingSettling.holdMs)ui.pendingSettling=null;
  $('positionSettlingApplyBtn').disabled=!ready||!p||other||ui.applyingTuning;
  $('positionSettlingState').textContent=p?`${ui.pendingSettling?'等待确认 · ': '设备设置 · '}误差 ±${p.toleranceDeg.toFixed(3)}°，连续 ${p.holdMs} ms；${[0,1,2].map(i=>`M${i+1} ${positionArrivalText(ui.latest,i)}`).join(' / ')}`:'稳定到位判定需要重新烧录V9固件';
  if(angleTest.run?.phase==='tracking'&&p?.state.every(s=>s===2))$('positionControlState').textContent='三路成功到位 · 外环暂停纠偏，超出范围自动恢复 · Q急停';
}
for(const id of ['positionTolerance','positionHoldMs'])$(id).oninput=stashPositionSettling;
$('positionSettlingApplyBtn').onclick=()=>{
  try{
    if(!ui.latest?.angleSettling)throw new Error('到位判定设置需要V9固件');
    const toleranceDeg=Number($('positionTolerance').value),holdMs=Number($('positionHoldMs').value);
    if(!Number.isFinite(toleranceDeg)||toleranceDeg<.001||toleranceDeg>10||!Number.isInteger(holdMs)||holdMs<1||holdMs>5000)throw new Error('误差范围0.001～10°，稳定时间1～5000整数ms');
    ui.pendingSettling={toleranceDeg:Math.round(toleranceDeg*1000)/1000,holdMs};stashPositionSettling();
    sendCommand(`H,${Math.round(toleranceDeg*1000)},${holdMs}`);notify('已下发稳定到位设置；重新开始连续稳定判定，不启动控制。');
  }catch(error){notify(error.message);}
};

$('positionMotorSelect').onchange=$('positionDirectionSelect').onchange=()=>{loadPositionDraft();renderStatus();};
for(const id of ['positionKp','positionKi','positionKd','positionMaxSpeed','positionMinSpeed'])$(id).oninput=stashPositionDraft;
$('positionApplyBtn').onclick=()=>{
  try{
    if(!ui.latest?.anglePid)throw new Error('角度PID配置需要V8固件');
    stashPositionDraft();const i=ui.positionLoadedMotor,d=ui.positionLoadedDirection,c=Tuning.normalize(ensureTuningDraft()),p=c.position.pid[i],g=p[d<0?'up':'down'];
    if(p.minSpeed!=null&&ui.latest.anglePid[i].minSpeed==null)throw new Error('最小角速度设置需要V11固件');
    ui.pendingPosition={motor:i,direction:d,...g,maxSpeed:p.maxSpeed,...(p.minSpeed==null?{}:{minSpeed:p.minSpeed})};
    sendCommand(`J,${i+1},${d},${[g.kp,g.ki,g.kd,p.maxSpeed,...(p.minSpeed==null?[]:[p.minSpeed])].map(v=>Math.round(v*1000)).join(',')}`);
    notify('角度PID与速度上下限已下发，等待设备回读；未启动角度环。');
  }catch(error){notify(error.message);}
};
$('positionStartBtn').onclick=()=>{
  try{if(test.run||fixedTest.run||speedTest.run)throw new Error('请先结束其他控制模式');angleTest.start([1,2,3].map(i=>Number($(`positionTarget${i}`).value)));$('positionTrackWindow').hidden=false;drawPositionTracking();notify('已请求三路目标角度，外环输出目标速度，速度环输出电流。');}
  catch(error){notify(error.message);}renderStatus();
};
$('positionStopBtn').onclick=()=>angleTest.finish('手动停止角度环');
attachSpeedWindow('positionParamsWindow','positionParamsHandle','positionParamsOpenBtn','positionParamsCloseBtn',()=>angleTest.finish('关闭角度调参浮窗'));
attachSpeedWindow('positionTrackWindow','positionTrackHandle','positionTrackOpenBtn','positionTrackCloseBtn');

draw(); renderStatus();

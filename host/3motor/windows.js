(function (factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else api.install(document, window, draw, attachSpeedWindow, ui.charts.response);
})(function () {
  'use strict';
  const geometryKeys = ['left', 'top', 'right', 'bottom', 'width', 'height'];

  function boundedSize(width, height, rect, viewport, minimum) {
    const maxWidth = Math.max(1, viewport.width - Math.max(0, rect.left) - 12);
    const maxHeight = Math.max(1, viewport.height - Math.max(viewport.header + 8, rect.top) - 12);
    return {width: Math.min(maxWidth, Math.max(Math.min(minimum.width, maxWidth), width)),
      height: Math.min(maxHeight, Math.max(Math.min(minimum.height, maxHeight), height))};
  }

  function install(doc, win, redraw, attachDrag, responseChart) {
    const byId = id => doc.getElementById(id);
    const ids = ['fixedWindow', 'speedParamsWindow', 'speedTrackWindow', 'positionParamsWindow', 'positionTrackWindow', 'responsePanel'];
    const panels = ids.map(byId), response = byId('responsePanel');
    const header = () => doc.querySelector('.topbar')?.getBoundingClientRect().bottom || 0;
    const floating = panel => panel.id !== 'responsePanel' || panel.classList.contains('response-floating');
    const geometry = panel => Object.fromEntries(geometryKeys.map(key => [key, panel.style[key]]));
    const minimum = panel => ({width: panel.id === 'fixedWindow' ? 280 : 360, height: 240});
    let drag = null, inlineStyle = null, floatingStyle = null;

    function resize(panel, width, height) {
      const size = boundedSize(width, height, panel.getBoundingClientRect(),
        {width: win.innerWidth, height: win.innerHeight, header: header()}, minimum(panel));
      Object.assign(panel.style, {width: `${size.width}px`, height: `${size.height}px`});
      redraw();
    }

    function reflow() {
      doc.documentElement.style.setProperty('--window-header-height', `${header()}px`);
      for (const panel of panels) {
        if (panel.hidden || !floating(panel) || panel.classList.contains('plot-maximized')) continue;
        const rect = panel.getBoundingClientRect();
        if (rect.top < header() + 8 || rect.top > win.innerHeight - 100) {
          panel.style.top = `${Math.max(header() + 8, Math.min(rect.top, win.innerHeight - 180))}px`;
          panel.style.bottom = 'auto';
        }
        if (rect.left > win.innerWidth - 100) { panel.style.left = `${Math.max(12, win.innerWidth - 100)}px`; panel.style.right = 'auto'; }
        if (panel.style.width || panel.style.height) resize(panel, rect.width, rect.height);
      }
      redraw();
    }

    // Reuse the existing document-level dragging, without moving the inline chart.
    attachDrag('responsePanel', 'responseHandle', 'responseOpenBtn', 'responseCloseBtn', null,
      () => response.classList.contains('response-floating'));

    function openResponse() {
      if (!floating(response)) {
        inlineStyle = geometry(response);
        response.classList.remove('plot-maximized');
        responseChart.expanded = false;
        byId('responseExpandBtn').textContent = '放大图表';
        response.classList.add('response-floating');
        Object.assign(response.style, floatingStyle || {left: '32px', top: `${header() + 12}px`, right: 'auto', bottom: 'auto',
          width: `${Math.min(1000, win.innerWidth - 48)}px`, height: `${Math.min(780, win.innerHeight - header() - 24)}px`});
        response.setAttribute('role', 'dialog');
        response.setAttribute('aria-modal', 'false');
        response.setAttribute('aria-labelledby', 'responseTitle');
      }
      response.hidden = false;
      byId('responseCloseBtn').hidden = false;
      byId('responseFloatBtn').hidden = true;
      reflow();
    }

    byId('responseOpenBtn').onclick = byId('responseFloatBtn').onclick = openResponse;
    byId('responseCloseBtn').onclick = () => {
      if (!floating(response)) return;
      floatingStyle = geometry(response);
      response.classList.remove('response-floating', 'plot-maximized');
      responseChart.expanded = false;
      Object.assign(response.style, inlineStyle);
      response.removeAttribute('role'); response.removeAttribute('aria-modal'); response.removeAttribute('aria-labelledby');
      response.hidden = false;
      byId('responseCloseBtn').hidden = true; byId('responseFloatBtn').hidden = false;
      byId('responseExpandBtn').textContent = '放大图表';
      byId('responseOpenBtn').focus({preventScroll: true});
      redraw();
    };

    for (const panel of panels) {
      const grip = byId(panel.id + 'Resize');
      grip.addEventListener('pointerdown', event => {
        if (event.button !== 0 || !floating(panel) || panel.classList.contains('plot-maximized')) return;
        event.preventDefault(); event.stopPropagation();
        const rect = panel.getBoundingClientRect();
        drag = {panel, id: event.pointerId, x: event.clientX, y: event.clientY, width: rect.width, height: rect.height};
      });
      grip.addEventListener('keydown', event => {
        if (!floating(panel) || panel.classList.contains('plot-maximized')) return;
        const amount = event.shiftKey ? 8 : 24;
        const delta = {ArrowLeft: [-amount, 0], ArrowRight: [amount, 0], ArrowUp: [0, -amount], ArrowDown: [0, amount]}[event.key];
        if (!delta) return;
        event.preventDefault(); event.stopPropagation();
        const rect = panel.getBoundingClientRect(); resize(panel, rect.width + delta[0], rect.height + delta[1]);
      });
    }
    doc.addEventListener('pointermove', event => {
      if (!drag || drag.id !== event.pointerId) return;
      resize(drag.panel, drag.width + event.clientX - drag.x, drag.height + event.clientY - drag.y);
    }, true);
    const end = event => { if (drag?.id === event.pointerId) drag = null; };
    doc.addEventListener('pointerup', end, true); doc.addEventListener('pointercancel', end, true);
    win.addEventListener('blur', () => {drag = null;});
    win.addEventListener('resize', reflow);
    for (const id of ['fixedOpenBtn', 'speedParamsOpenBtn', 'speedTrackOpenBtn', 'positionParamsOpenBtn', 'positionTrackOpenBtn']) byId(id).addEventListener('click', reflow);
    if (typeof win.ResizeObserver === 'function') {
      const observer = new win.ResizeObserver(reflow); observer.observe(doc.querySelector('.topbar'));
    }
    reflow();
  }
  return {boundedSize, install};
});

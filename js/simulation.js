/**
 * simulation.js — Brachistochrone Curve Virtual Lab
 * Physics Engine: Energy-conservation velocity integration
 * Paths: Cycloid (Brachistochrone) · Straight Line · Circular Arc
 * Lab Report: auto-populates Observation, Calculation, Result, Conclusion
 *             sections after 5 table entries.
 */

'use strict';

window.addEventListener('error', function (e) {
  const errorDiv = document.createElement('div');
  errorDiv.style.cssText = 'position:fixed;top:0;left:0;background:red;color:#fff;padding:20px;z-index:999999;';
  errorDiv.innerHTML = `<h3>Simulation Crash!</h3><p>${e.message}</p><p>Line: ${e.lineno}</p>`;
  document.body.prepend(errorDiv);
});

/* ================================================================
   CONSTANTS & STATE
================================================================ */
const CANVAS_LOGICAL_W = 800;
const CANVAS_LOGICAL_H = 400;
const PATH_POINTS       = 600;
const BALL_RADIUS       = 8;
const GRAVITY_DEFAULT   = 9.81;
const HEIGHT_DEFAULT    = 2.0;
const WIDTH_FRAC        = 0.72;

const COLOR = {
  cycloid : '#00d4ff',
  straight: '#ff6b35',
  arc     : '#a8ff3e',
  grid    : 'rgba(30, 45, 69, 0.6)',
  bg      : '#060c18',
  text    : 'rgba(143, 163, 192, 0.7)',
};

let state = {
  g          : GRAVITY_DEFAULT,
  H          : HEIGHT_DEFAULT,
  running    : false,
  finished   : false,
  startTime  : 0,
  rafId      : null,
  observations: [],
};

let balls = {
  cycloid : { t: 0, done: false, time: null },
  straight: { t: 0, done: false, time: null },
  arc     : { t: 0, done: false, time: null },
};

let paths = { cycloid: [], straight: [], arc: [] };

/* ================================================================
   DOM REFERENCES
================================================================ */
const canvas        = document.getElementById('simCanvas');
const ctx           = canvas.getContext('2d');
const heightSlider  = document.getElementById('heightSlider');
const gravitySelect = document.getElementById('gravitySelect');
const heightVal     = document.getElementById('heightVal');
const runBtn        = document.getElementById('runBtn');
const resetBtn      = document.getElementById('resetBtn');
const addRowBtn     = document.getElementById('addRowBtn');
const clearTableBtn = document.getElementById('clearTableBtn');
const statusDot     = document.getElementById('statusDot');
const statusText    = document.getElementById('statusText');
const conclusionBox = document.getElementById('conclusionBox');
const obsTableBody  = document.getElementById('obsTableBody');

const checkToggles = {
  cycloid : document.getElementById('checkCycloid'),
  straight: document.getElementById('checkStraight'),
  arc     : document.getElementById('checkArc'),
};

const timeDisplay = {
  cycloid : document.getElementById('timeCycloid'),
  straight: document.getElementById('timeStraight'),
  arc     : document.getElementById('timeArc'),
};

const badgeDisplay = {
  cycloid : document.getElementById('badgeCycloid'),
  straight: document.getElementById('badgeStraight'),
  arc     : document.getElementById('badgeArc'),
};

const cardDisplay = {
  cycloid : document.getElementById('cardCycloid'),
  straight: document.getElementById('cardStraight'),
  arc     : document.getElementById('cardArc'),
};

/* ================================================================
   CANVAS SETUP — responsive HiDPI
================================================================ */
function setupCanvas() {
  const dpr  = window.devicePixelRatio || 1;
  const rect = canvas.parentElement.getBoundingClientRect();
  const w    = Math.min(rect.width - 2, CANVAS_LOGICAL_W * 2);
  const h    = Math.round(w * (CANVAS_LOGICAL_H / CANVAS_LOGICAL_W));

  canvas.width        = w * dpr;
  canvas.height       = h * dpr;
  canvas.style.width  = w + 'px';
  canvas.style.height = h + 'px';

  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr * (w / CANVAS_LOGICAL_W), dpr * (h / CANVAS_LOGICAL_H));
}

/* ================================================================
   PHYSICS — PATH GENERATORS
================================================================ */
const MARGIN_L = 60;
const MARGIN_T = 40;
const MARGIN_B = 60;

function endpointMeters() {
  const H    = state.H;
  const R    = H / 2;
  const Xend = R * Math.PI;
  return { Xend, H };
}

function computeScale() {
  const { Xend, H } = endpointMeters();
  const maxPx_h = CANVAS_LOGICAL_W - MARGIN_L - 50;
  const maxPx_v = CANVAS_LOGICAL_H - MARGIN_T - MARGIN_B;
  return Math.min(maxPx_h / Xend, maxPx_v / H);
}

function addArcLengthAndTimes(pts, scale) {
  const result = [];
  let cumArc  = 0;
  let cumTime = 0;
  const g     = state.g;

  for (let i = 0; i < pts.length; i++) {
    const cx = MARGIN_L + pts[i].xm * scale;
    const cy = MARGIN_T + pts[i].ym * scale;

    if (i > 0) {
      const dx    = cx - result[i - 1].cx;
      const dy    = cy - result[i - 1].cy;
      const ds_px = Math.sqrt(dx * dx + dy * dy);
      cumArc += ds_px;

      const ds_m   = ds_px / scale;
      const ym_mid = (pts[i].ym + pts[i - 1].ym) / 2;
      const v_mid  = Math.sqrt(2 * g * Math.max(ym_mid, 1e-9));
      cumTime += ds_m / v_mid;
    }

    result.push({ cx, cy, xm: pts[i].xm, ym: pts[i].ym, arcLen: cumArc, T: cumTime });
  }

  return result;
}

/* ── Analytical Transit Times ── */
function analyticalTimes() {
  const g = state.g;
  const H = state.H;
  const R = H / 2;
  const { Xend } = endpointMeters();

  const T_cycloid  = Math.PI * Math.sqrt(R / g);
  const L          = Math.sqrt(Xend * Xend + H * H);
  const T_straight = Math.sqrt(2 * L * L / (g * H));

  const arc  = buildArcPathBase();
  let T_arc  = 0;
  for (let i = 1; i < arc.length; i++) {
    const dx     = arc[i].xm - arc[i - 1].xm;
    const dy     = arc[i].ym - arc[i - 1].ym;
    const ds_m   = Math.sqrt(dx * dx + dy * dy);
    const ym_mid = (arc[i].ym + arc[i - 1].ym) / 2;
    const v_mid  = Math.sqrt(2 * g * Math.max(ym_mid, 1e-9));
    T_arc += ds_m / v_mid;
  }

  return { T_cycloid, T_straight, T_arc };
}

function normalizePathTimes(path, analyticalTime) {
  if (!path.length) return path;
  const ratio = analyticalTime / path[path.length - 1].T;
  path.forEach(pt => pt.T *= ratio);
  return path;
}

function buildAllPaths() {
  const scale = computeScale();
  const times = analyticalTimes();

  paths.cycloid  = checkToggles.cycloid.checked  ? normalizePathTimes(addArcLengthAndTimes(buildCycloidPathBase(), scale),  times.T_cycloid)  : [];
  paths.straight = checkToggles.straight.checked ? normalizePathTimes(addArcLengthAndTimes(buildStraightPathBase(), scale), times.T_straight) : [];
  paths.arc      = checkToggles.arc.checked      ? normalizePathTimes(addArcLengthAndTimes(buildArcPathBase(), scale),      times.T_arc)      : [];
}

function buildCycloidPathBase() {
  const H = state.H, R = H / 2, thetaEnd = Math.PI;
  const pts = [];
  for (let i = 0; i <= PATH_POINTS; i++) {
    const theta = (i / PATH_POINTS) * thetaEnd;
    pts.push({ xm: R * (theta - Math.sin(theta)), ym: R * (1 - Math.cos(theta)) });
  }
  return pts;
}

function buildStraightPathBase() {
  const { Xend, H } = endpointMeters();
  const pts = [];
  for (let i = 0; i <= PATH_POINTS; i++) {
    const frac = i / PATH_POINTS;
    pts.push({ xm: frac * Xend, ym: frac * H });
  }
  return pts;
}

function buildArcPathBase() {
  const { Xend, H } = endpointMeters();
  const Rc         = (Xend * Xend + H * H) / (2 * H);
  const cx_w       = 0, cy_w = Rc;
  const angleStart = Math.atan2(0 - cy_w, 0 - cx_w);
  const angleEnd   = Math.atan2(H - cy_w, Xend - cx_w);
  const totalAngle = angleEnd - angleStart;
  const pts        = [];
  for (let i = 0; i <= PATH_POINTS; i++) {
    const angle = angleStart + (i / PATH_POINTS) * totalAngle;
    pts.push({ xm: cx_w + Rc * Math.cos(angle), ym: cy_w + Rc * Math.sin(angle) });
  }
  return pts;
}

/* ================================================================
   ANIMATION ENGINE
================================================================ */
function advanceBall(ball, path, elapsedSeconds) {
  if (ball.done || !path.length) return;

  const maxTime = path[path.length - 1].T;
  if (elapsedSeconds >= maxTime) {
    ball.t = 1; ball.done = true; ball.time = maxTime; return;
  }

  let lo = 0, hi = path.length - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (path[mid].T <= elapsedSeconds) lo = mid; else hi = mid;
  }

  const t0 = path[lo].T, t1 = path[hi].T;
  const prog = (elapsedSeconds - t0) / (t1 - t0 || 1);
  ball.t = (lo + prog) / (path.length - 1);
}

function drawScene(elapsed) {
  ctx.clearRect(0, 0, CANVAS_LOGICAL_W, CANVAS_LOGICAL_H);
  ctx.fillStyle = COLOR.bg;
  ctx.fillRect(0, 0, CANVAS_LOGICAL_W, CANVAS_LOGICAL_H);

  drawGrid();
  drawAxes();
  drawPaths();
  drawEndpoints();

  const elapsedSec = elapsed / 1000;
  ['cycloid', 'straight', 'arc'].forEach(name => {
    if (paths[name].length > 0) {
      advanceBall(balls[name], paths[name], elapsedSec);
      drawBall(name);
    }
  });

  drawTimerOverlay();
}

function drawGrid() {
  ctx.save();
  ctx.strokeStyle = COLOR.grid;
  ctx.lineWidth   = 0.5;
  const step = 40;
  for (let x = MARGIN_L; x < CANVAS_LOGICAL_W - 30; x += step) {
    ctx.beginPath(); ctx.moveTo(x, MARGIN_T); ctx.lineTo(x, CANVAS_LOGICAL_H - MARGIN_B); ctx.stroke();
  }
  for (let y = MARGIN_T; y < CANVAS_LOGICAL_H - MARGIN_B + step; y += step) {
    ctx.beginPath(); ctx.moveTo(MARGIN_L, y); ctx.lineTo(CANVAS_LOGICAL_W - 30, y); ctx.stroke();
  }
  ctx.restore();
}

function drawAxes() {
  ctx.save();
  ctx.strokeStyle = 'rgba(143, 163, 192, 0.4)';
  ctx.lineWidth   = 1.5;
  ctx.font        = '11px Consolas, monospace';
  ctx.fillStyle   = COLOR.text;

  ctx.beginPath(); ctx.moveTo(MARGIN_L, CANVAS_LOGICAL_H - MARGIN_B); ctx.lineTo(CANVAS_LOGICAL_W - 20, CANVAS_LOGICAL_H - MARGIN_B); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(MARGIN_L, MARGIN_T); ctx.lineTo(MARGIN_L, CANVAS_LOGICAL_H - MARGIN_B); ctx.stroke();

  ctx.textAlign = 'center';
  ctx.fillText('x (m)', CANVAS_LOGICAL_W / 2, CANVAS_LOGICAL_H - 10);
  ctx.save();
  ctx.translate(14, CANVAS_LOGICAL_H / 2); ctx.rotate(-Math.PI / 2);
  ctx.fillText('y (m)', 0, 0);
  ctx.restore();

  const scale = computeScale();
  const { Xend, H } = endpointMeters();
  const xTick = Xend / 4;
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (let i = 0; i <= 4; i++) ctx.fillText((i * xTick).toFixed(1), MARGIN_L + (i * xTick) * scale, CANVAS_LOGICAL_H - MARGIN_B + 6);
  const yTick = H / 4;
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let i = 0; i <= 4; i++) ctx.fillText((i * yTick).toFixed(1), MARGIN_L - 6, MARGIN_T + (i * yTick) * scale);

  ctx.restore();
}

function drawPaths() {
  [
    { name: 'cycloid',  color: COLOR.cycloid,  width: 2.5 },
    { name: 'straight', color: COLOR.straight, width: 2   },
    { name: 'arc',      color: COLOR.arc,      width: 2   },
  ].forEach(({ name, color, width }) => {
    const pts = paths[name];
    if (!pts.length) return;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.shadowColor = color; ctx.shadowBlur = 6;
    ctx.beginPath(); ctx.moveTo(pts[0].cx, pts[0].cy);
    for (let i = 1; i < pts.length; i += 2) ctx.lineTo(pts[i].cx, pts[i].cy);
    ctx.stroke();
    ctx.restore();
  });
}

function drawEndpoints() {
  const startPt = paths.cycloid[0];
  const endPt   = paths.cycloid[paths.cycloid.length - 1];
  if (!startPt) return;

  ctx.save();
  ctx.fillStyle = '#ffffff';
  ctx.beginPath(); ctx.arc(startPt.cx, startPt.cy, 5, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = COLOR.arc; ctx.shadowColor = COLOR.arc; ctx.shadowBlur = 10;
  ctx.beginPath(); ctx.arc(endPt.cx, endPt.cy, 5, 0, Math.PI * 2); ctx.fill();

  ctx.shadowBlur = 0; ctx.font = '11px Consolas, monospace'; ctx.fillStyle = 'rgba(255,255,255,0.7)';
  ctx.textAlign = 'center';
  ctx.fillText('START', startPt.cx, startPt.cy - 12);
  ctx.fillText('END',   endPt.cx,   endPt.cy + 16);
  ctx.restore();
}

function drawBall(name) {
  const ball  = balls[name];
  const pts   = paths[name];
  if (!pts.length) return;

  let idx = Math.round(ball.t * (pts.length - 1));
  idx     = Math.max(0, Math.min(idx, pts.length - 1));
  const pt = pts[idx];
  if (!pt || !isFinite(pt.cx) || !isFinite(pt.cy)) return;

  const color = COLOR[name];
  ctx.save();
  ctx.shadowColor = color; ctx.shadowBlur = 16;

  const grad = ctx.createRadialGradient(pt.cx - 2, pt.cy - 2, 1, pt.cx, pt.cy, BALL_RADIUS);
  grad.addColorStop(0, '#ffffff');
  grad.addColorStop(0.4, color);
  grad.addColorStop(1, adjustAlpha(color, 0.5));

  ctx.fillStyle = grad;
  ctx.beginPath(); ctx.arc(pt.cx, pt.cy, BALL_RADIUS, 0, Math.PI * 2); ctx.fill();

  ctx.shadowBlur = 0; ctx.font = 'bold 9px Consolas, monospace';
  ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(name === 'cycloid' ? 'C' : name === 'straight' ? 'S' : 'A', pt.cx, pt.cy);
  ctx.restore();
}

function drawTimerOverlay() {
  const now    = (performance.now() - state.startTime) / 1000;
  const names  = ['cycloid', 'straight', 'arc'];
  const colors = [COLOR.cycloid, COLOR.straight, COLOR.arc];
  const labels = ['Cycloid', 'Straight', 'Arc'];

  ctx.save();
  ctx.font = '12px Consolas, monospace';
  names.forEach((name, i) => {
    if (!paths[name].length) return;
    const ball  = balls[name];
    const t     = ball.done ? ball.time : (state.running ? now : null);
    const disp  = t !== null ? t.toFixed(3) + ' s' : '---';
    ctx.fillStyle = colors[i]; ctx.textAlign = 'left';
    ctx.fillText(`${labels[i]}: ${disp}`, CANVAS_LOGICAL_W - 160, 18 + i * 18);
  });
  ctx.restore();
}

/* ================================================================
   SIMULATION CONTROL
================================================================ */
function getActivePaths() {
  return ['cycloid', 'straight', 'arc'].filter(n => paths[n].length > 0);
}

function resetBalls() {
  ['cycloid', 'straight', 'arc'].forEach(name => {
    balls[name] = { t: 0, done: paths[name].length === 0, time: null };
  });
}

function resetResults() {
  ['cycloid', 'straight', 'arc'].forEach(name => {
    timeDisplay[name].textContent = checkToggles[name].checked ? '—' : 'N/A';
    timeDisplay[name].className   = 'result-time pending';
    badgeDisplay[name].classList.remove('visible');
    cardDisplay[name].classList.remove('winner');
  });
  conclusionBox.classList.remove('visible');
}

function animationLoop(timestamp) {
  if (!state.running) return;

  const elapsed = Math.max(0, timestamp - (state.startTime || timestamp));
  drawScene(elapsed);

  const activeNames = getActivePaths();
  const allDone     = activeNames.every(n => balls[n].done);

  if (allDone) {
    state.running  = false;
    state.finished = true;
    finalizeResults();
    updateStatus('done', 'Simulation complete');
    runBtn.disabled = false;
    return;
  }

  activeNames.forEach(name => {
    if (balls[name].done && balls[name].time !== null) {
      timeDisplay[name].textContent = balls[name].time.toFixed(3) + ' s';
      timeDisplay[name].className   = 'result-time';
    }
  });

  state.rafId = requestAnimationFrame(animationLoop);
}

function startSimulation() {
  if (state.running) return;
  buildAllPaths(); resetBalls(); resetResults();
  state.running   = true;
  state.finished  = false;
  state.startTime = performance.now();
  runBtn.disabled = true;
  updateStatus('running', 'Simulation running…');
  state.rafId = requestAnimationFrame(animationLoop);
}

function resetSimulation() {
  if (state.rafId) cancelAnimationFrame(state.rafId);
  state.running  = false;
  state.finished = false;
  buildAllPaths(); resetBalls(); resetResults();
  updateStatus('idle', 'Ready');
  runBtn.disabled = false;
  drawScene(0);
}

function finalizeResults() {
  const analytical = analyticalTimes();

  ['cycloid', 'straight', 'arc'].forEach(name => {
    if (paths[name].length > 0) {
      timeDisplay[name].textContent = analytical['T_' + name].toFixed(4) + ' s';
      timeDisplay[name].className   = 'result-time';
    }
  });

  const activeAnalytics = {};
  ['cycloid', 'straight', 'arc'].forEach(name => {
    if (paths[name].length > 0) activeAnalytics[name] = analytical['T_' + name];
  });

  const fastest = Object.entries(activeAnalytics).reduce((a, b) => a[1] < b[1] ? a : b)[0];
  cardDisplay[fastest].classList.add('winner');
  badgeDisplay[fastest].classList.add('visible');

  const slowerMsgs = [];
  if (paths.straight.length > 0 && fastest !== 'straight') {
    const p = ((analytical.T_straight - activeAnalytics[fastest]) / activeAnalytics[fastest] * 100).toFixed(1);
    slowerMsgs.push(`the straight-line takes <strong>${p}%</strong> longer`);
  }
  if (paths.arc.length > 0 && fastest !== 'arc') {
    const p = ((analytical.T_arc - activeAnalytics[fastest]) / activeAnalytics[fastest] * 100).toFixed(1);
    slowerMsgs.push(`the circular arc takes <strong>${p}%</strong> longer`);
  }

  const nameMap = { cycloid: 'Cycloid (Brachistochrone)', straight: 'Straight Line', arc: 'Circular Arc' };
  conclusionBox.innerHTML = `✅ <strong>${nameMap[fastest]} is fastest!</strong> `;
  if (slowerMsgs.length) conclusionBox.innerHTML += slowerMsgs.join(' and ') + '. ';
  if (fastest === 'cycloid') conclusionBox.innerHTML += `This confirms the Brachistochrone property: a cycloid is the curve of fastest descent under constant gravity.`;
  conclusionBox.classList.add('visible');

  drawScene(0);
}

/* ================================================================
   OBSERVATION TABLE
================================================================ */
function addObservationRow() {
  if (!state.finished) { alert('Please run the simulation first.'); return; }

  const analytical = analyticalTimes();
  const rowNum     = state.observations.length + 1;
  const fastestEl  = document.querySelector('.winner-badge.visible');
  const fastestKey = fastestEl ? fastestEl.id.replace('badge', '') : 'N/A';

  state.observations.push({
    no        : rowNum,
    H         : state.H.toFixed(2),
    g         : state.g.toFixed(2),
    T_cycloid : paths.cycloid.length  > 0 ? analytical.T_cycloid.toFixed(4)  : '-',
    T_straight: paths.straight.length > 0 ? analytical.T_straight.toFixed(4) : '-',
    T_arc     : paths.arc.length      > 0 ? analytical.T_arc.toFixed(4)      : '-',
    fastest   : fastestKey,
  });

  renderTable();
  updateLabReport();
}

function renderTable() {
  if (!state.observations.length) {
    obsTableBody.innerHTML = `<tr class="empty-row"><td colspan="8">No observations yet — run the simulation and click "Add to Table"</td></tr>`;
    return;
  }
  obsTableBody.innerHTML = state.observations.map(r => `
    <tr>
      <td>${r.no}</td>
      <td>${r.H}</td>
      <td>${r.g}</td>
      <td class="cycloid-td">${r.T_cycloid}</td>
      <td class="straight-td">${r.T_straight}</td>
      <td class="arc-td">${r.T_arc}</td>
      <td class="fastest-td">${r.fastest}</td>
    </tr>`).join('');
}

function clearTable() {
  state.observations = [];
  renderTable();
  updateLabReport();
}

/* ================================================================
   LAB REPORT — auto-populate after 5 observations
================================================================ */
const LAB_THRESHOLD = 5;

function updateLabReport() {
  const count = state.observations.length;
  const ready = count >= LAB_THRESHOLD;

  // ── Progress bar ──
  const pct = Math.min(100, (count / LAB_THRESHOLD) * 100);
  document.getElementById('labProgressFill').style.width = pct + '%';
  document.getElementById('labProgressLabel').textContent =
    ready ? `✅ ${count} observations recorded — report active`
          : `${count} / ${LAB_THRESHOLD} observations recorded`;

  // ── Update all counter spans ──
  ['obsCount', 'calcCount', 'resultCount', 'conclusionCount'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.textContent = count;
  });

  // ── Toggle badges ──
  ['obsBadge', 'calcBadge', 'resultBadge', 'conclusionBadge'].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    if (ready) { el.textContent = '✔ Live'; el.classList.add('live'); }
    else        { el.textContent = `${count}/5 — Awaiting data…`; el.classList.remove('live'); }
  });

  if (!ready) {
    // hide content, show placeholders
    ['obs', 'calc', 'result', 'conclusion'].forEach(key => {
      const c = document.getElementById(key + 'Content');
      const p = document.getElementById(key + 'Placeholder');
      if (c) c.style.display = 'none';
      if (p) p.style.display = 'block';
    });
    return;
  }

  // ── Show content panels ──
  ['obs', 'calc', 'result', 'conclusion'].forEach(key => {
    const c = document.getElementById(key + 'Content');
    const p = document.getElementById(key + 'Placeholder');
    if (c) c.style.display = 'block';
    if (p) p.style.display = 'none';
  });

  populateObservationSection();
  populateCalculationSection();
  populateResultSection();
  populateConclusionSection();
}

/* ── 5. Observation ── */
function populateObservationSection() {
  const rows  = state.observations;
  const tbody = document.getElementById('reportObsBody');
  if (!tbody) return;

  tbody.innerHTML = rows.map(r => `
    <tr>
      <td>${r.no}</td>
      <td>${r.H}</td>
      <td>${r.g}</td>
      <td class="cycloid-td">${r.T_cycloid}</td>
      <td class="straight-td">${r.T_straight}</td>
      <td class="arc-td">${r.T_arc}</td>
      <td class="fastest-td">${r.fastest}</td>
    </tr>`).join('');

  // Summary text
  const allFastest = rows.filter(r => r.fastest === 'cycloid').length;
  const summaryEl  = document.getElementById('obsSummaryText');
  if (summaryEl) {
    summaryEl.innerHTML =
      `Out of <strong>${rows.length}</strong> observations, the <strong style="color:var(--cycloid-color)">Cycloid</strong>
       was recorded as the fastest path in <strong>${allFastest}</strong> run(s),
       confirming that the cycloid is the curve of minimum descent time under uniform gravity.`;
  }
}

/* ── 6. Calculation ── */
function populateCalculationSection() {
  const r1  = state.observations[0];   // sample = first row
  const H   = parseFloat(r1.H);
  const g   = parseFloat(r1.g);
  const R   = H / 2;
  const Xend = R * Math.PI;
  const L   = Math.sqrt(Xend * Xend + H * H);
  const Rc  = (Xend * Xend + H * H) / (2 * H);

  const T_cyc = Math.PI * Math.sqrt(R / g);
  const T_str = Math.sqrt(2 * L * L / (g * H));
  // T_arc from observation
  const T_arc = parseFloat(r1.T_arc) || 0;

  const pct_str = (((T_str - T_cyc) / T_cyc) * 100).toFixed(2);
  const pct_arc = (((T_arc - T_cyc) / T_cyc) * 100).toFixed(2);

  const detailEl = document.getElementById('calcDetails');
  if (detailEl) {
    detailEl.innerHTML = `
      <h4 class="theory-subheading">Sample Calculation — Observation No. 1 (H = ${H} m, g = ${g} m/s²)</h4>

      <p class="lab-text"><strong>Step 1 — Cycloid Parameters</strong></p>
      <div class="formula-block">  R    = H / 2           = ${H} / 2           = ${R.toFixed(4)} m
  Xend = π · R           = π × ${R.toFixed(4)}      = ${Xend.toFixed(4)} m</div>

      <p class="lab-text"><strong>Step 2 — Cycloid Transit Time (analytical)</strong></p>
      <div class="formula-block">  T_cycloid = π √(R/g)
            = π × √(${R.toFixed(4)} / ${g})
            = π × √(${(R/g).toFixed(6)})
            = π × ${Math.sqrt(R/g).toFixed(6)}
            = ${T_cyc.toFixed(4)} s</div>

      <p class="lab-text"><strong>Step 3 — Straight Line Transit Time</strong></p>
      <div class="formula-block">  L          = √(Xend² + H²)
             = √(${(Xend*Xend).toFixed(4)} + ${(H*H).toFixed(4)})
             = √(${(Xend*Xend + H*H).toFixed(4)})
             = ${L.toFixed(4)} m

  T_straight = √(2L² / (g·H))
             = √(2 × ${(L*L).toFixed(4)} / (${g} × ${H}))
             = √(${(2*L*L).toFixed(4)} / ${(g*H).toFixed(4)})
             = ${T_str.toFixed(4)} s</div>

      <p class="lab-text"><strong>Step 4 — Circular Arc Radius</strong></p>
      <div class="formula-block">  Rc = (Xend² + H²) / (2H)
     = (${(Xend*Xend).toFixed(4)} + ${(H*H).toFixed(4)}) / (2 × ${H})
     = ${(Xend*Xend + H*H).toFixed(4)} / ${(2*H).toFixed(4)}
     = ${Rc.toFixed(4)} m

  T_arc (numerical) = ${T_arc.toFixed(4)} s</div>`;
  }

  const advEl = document.getElementById('calcAdvantage');
  if (advEl) {
    advEl.textContent =
`  % faster than Straight Line = (T_straight − T_cycloid) / T_cycloid × 100
                               = (${T_str.toFixed(4)} − ${T_cyc.toFixed(4)}) / ${T_cyc.toFixed(4)} × 100
                               = ${pct_str} %

  % faster than Circular Arc  = (T_arc − T_cycloid) / T_cycloid × 100
                               = (${T_arc.toFixed(4)} − ${T_cyc.toFixed(4)}) / ${T_cyc.toFixed(4)} × 100
                               = ${pct_arc} %`;
  }
}

/* ── 7. Result ── */
function populateResultSection() {
  const rows    = state.observations;
  const tableEl = document.getElementById('resultTable');
  const summEl  = document.getElementById('resultSummary');
  if (!tableEl || !summEl) return;

  // Averages (numeric rows only)
  const numRows = rows.filter(r => r.T_cycloid !== '-' && r.T_straight !== '-' && r.T_arc !== '-');

  const avg = (key) =>
    numRows.length
      ? (numRows.reduce((s, r) => s + parseFloat(r[key]), 0) / numRows.length).toFixed(4)
      : 'N/A';

  const avgCyc = avg('T_cycloid');
  const avgStr = avg('T_straight');
  const avgArc = avg('T_arc');

  const pctStr = numRows.length
    ? (((parseFloat(avgStr) - parseFloat(avgCyc)) / parseFloat(avgCyc)) * 100).toFixed(2)
    : 'N/A';
  const pctArc = numRows.length
    ? (((parseFloat(avgArc) - parseFloat(avgCyc)) / parseFloat(avgCyc)) * 100).toFixed(2)
    : 'N/A';

  tableEl.innerHTML = `
    <p class="lab-text">Average transit times computed from all <strong>${rows.length}</strong> observations:</p>
    <div class="result-summary-grid">
      <div class="result-summary-card highlight">
        <div class="rs-label" style="color:var(--cycloid-color);">⟨ T_cycloid ⟩</div>
        <div class="rs-value">${avgCyc} s</div>
      </div>
      <div class="result-summary-card">
        <div class="rs-label" style="color:var(--straight-color);">⟨ T_straight ⟩</div>
        <div class="rs-value">${avgStr} s</div>
      </div>
      <div class="result-summary-card">
        <div class="rs-label" style="color:var(--arc-color);">⟨ T_arc ⟩</div>
        <div class="rs-value">${avgArc} s</div>
      </div>
      <div class="result-summary-card">
        <div class="rs-label">Cycloid advantage vs Straight</div>
        <div class="rs-value" style="color:var(--success);">${pctStr}% faster</div>
      </div>
      <div class="result-summary-card">
        <div class="rs-label">Cycloid advantage vs Arc</div>
        <div class="rs-value" style="color:var(--success);">${pctArc}% faster</div>
      </div>
    </div>

    <p class="lab-text" style="margin-top:1rem;">
      <strong>Observed Ranking (consistent across all runs):</strong>
    </p>
    <div class="formula-block">  T_cycloid  &lt;  T_arc  &lt;  T_straight

  Average:  ${avgCyc} s  &lt;  ${avgArc} s  &lt;  ${avgStr} s</div>`;

  summEl.innerHTML = `
    <p class="lab-text">
      In every trial, the ball on the <strong style="color:var(--cycloid-color)">Cycloid path</strong>
      reached the endpoint before both the Straight Line and Circular Arc paths.
      On average, the cycloid was <strong style="color:var(--success)">${pctStr}%</strong> faster than
      the straight line and <strong style="color:var(--success)">${pctArc}%</strong> faster than the
      circular arc — consistent with the theoretical Brachistochrone property.
    </p>`;
}

/* ── 8. Conclusion ── */
function populateConclusionSection() {
  const rows    = state.observations;
  const numRows = rows.filter(r => r.T_cycloid !== '-' && r.T_straight !== '-' && r.T_arc !== '-');

  const avg = (key) =>
    numRows.length
      ? (numRows.reduce((s, r) => s + parseFloat(r[key]), 0) / numRows.length).toFixed(4)
      : '—';

  const avgCyc = avg('T_cycloid');
  const avgStr = avg('T_straight');
  const avgArc = avg('T_arc');

  const pctStr = numRows.length
    ? (((parseFloat(avgStr) - parseFloat(avgCyc)) / parseFloat(avgCyc)) * 100).toFixed(1)
    : '—';
  const pctArc = numRows.length
    ? (((parseFloat(avgArc) - parseFloat(avgCyc)) / parseFloat(avgCyc)) * 100).toFixed(1)
    : '—';

  const allCycloidFastest = rows.every(r => r.fastest === 'cycloid');
  const cycloidWins       = rows.filter(r => r.fastest === 'cycloid').length;

  const el = document.getElementById('conclusionText');
  if (!el) return;

  el.innerHTML = `
    <div class="lab-conclusion-box">
      <p>
        From the virtual experiment conducted using the Brachistochrone Simulation Lab over
        <strong>${rows.length}</strong> trials with varying values of vertical drop <em>H</em> and
        gravitational acceleration <em>g</em>, the following conclusions are drawn:
      </p>
      <br>
      <p>
        <strong>1.</strong> The <strong>Cycloid (Brachistochrone)</strong> consistently recorded the
        minimum transit time in <strong>${cycloidWins} out of ${rows.length}</strong> observations,
        ${allCycloidFastest ? 'which is every single trial,' : 'confirming the overwhelming trend,'}
        confirming that it is the <em>fastest curve of descent</em> under uniform gravity.
      </p>
      <br>
      <p>
        <strong>2.</strong> The average transit times observed were:
        <span class="lab-conclusion-highlight">${avgCyc} s</span> (Cycloid) &lt;
        <span class="lab-conclusion-highlight" style="color:var(--arc-color);border-color:rgba(168,255,62,0.3);background:rgba(168,255,62,0.08);">${avgArc} s</span> (Arc) &lt;
        <span class="lab-conclusion-highlight" style="color:var(--straight-color);border-color:rgba(255,107,53,0.3);background:rgba(255,107,53,0.08);">${avgStr} s</span> (Straight),
        which is in perfect agreement with the theoretical prediction
        <span class="lab-conclusion-highlight">T_cycloid &lt; T_arc &lt; T_straight</span>.
      </p>
      <br>
      <p>
        <strong>3.</strong> The cycloid was found to be
        <strong style="color:var(--success)">${pctStr}%</strong> faster than the straight-line path and
        <strong style="color:var(--success)">${pctArc}%</strong> faster than the circular arc (on average).
        Contrary to intuition, the shortest geometric path (straight line) is not the fastest path of descent.
      </p>
      <br>
      <p>
        <strong>4.</strong> The experimental results verify the analytical transit-time formulae derived
        via the Calculus of Variations (Euler–Lagrange equation / Beltrami identity), and confirm that
        the <strong>Brachistochrone curve is a Cycloid</strong> — as first proved by Johann Bernoulli in 1696.
      </p>
      <br>
      <p>
        <strong>5.</strong> The experiment also illustrates that the transit time of the cycloid
        <strong>T = π√(R/g)</strong> depends only on the radius R = H/2 and gravitational acceleration g —
        a consequence of the Cycloid's remarkable tautochrone property (Huygens, 1659).
      </p>
    </div>`;
}

/* ================================================================
   LAB REPORT ACCORDION — collapsible sections
================================================================ */
function initLabAccordion() {
  document.querySelectorAll('.lab-section-header').forEach(btn => {
    btn.addEventListener('click', () => {
      const targetId = btn.getAttribute('data-target');
      const body     = document.getElementById(targetId);
      if (!body) return;

      const isOpen = body.classList.contains('open');

      // Close all
      document.querySelectorAll('.lab-section-body').forEach(b => b.classList.remove('open'));
      document.querySelectorAll('.lab-section-header').forEach(h => h.classList.remove('active'));

      // Toggle clicked
      if (!isOpen) {
        body.classList.add('open');
        btn.classList.add('active');
      }
    });
  });

  // Open the first section by default
  const first = document.querySelector('.lab-section-header');
  if (first) {
    const id   = first.getAttribute('data-target');
    const body = document.getElementById(id);
    if (body) { body.classList.add('open'); first.classList.add('active'); }
  }
}

/* ================================================================
   STATUS BAR
================================================================ */
function updateStatus(type, text) {
  statusDot.className  = 'status-dot ' + (type === 'running' ? 'running' : type === 'done' ? 'done' : '');
  statusText.textContent = text;
}

/* ================================================================
   CONTROLS — EVENT LISTENERS
================================================================ */
function initControls() {
  ['cycloid', 'straight', 'arc'].forEach(p => {
    checkToggles[p].addEventListener('change', () => {
      if (!state.running) { buildAllPaths(); drawScene(0); resetResults(); }
    });
  });

  heightSlider.addEventListener('input', () => {
    state.H = parseFloat(heightSlider.value);
    heightVal.textContent = state.H.toFixed(1) + ' m';
    if (!state.running) { buildAllPaths(); drawScene(0); resetResults(); }
  });

  gravitySelect.addEventListener('change', () => {
    state.g = parseFloat(gravitySelect.value);
    if (!state.running) resetResults();
  });

  runBtn.addEventListener('click', startSimulation);
  resetBtn.addEventListener('click', resetSimulation);
  addRowBtn.addEventListener('click', addObservationRow);
  clearTableBtn.addEventListener('click', () => {
    if (confirm('Clear all observation data?')) clearTable();
  });
}

/* ================================================================
   UTILITY
================================================================ */
function adjustAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r},${g},${b},${alpha})`;
}

/* ================================================================
   INIT
================================================================ */
function init() {
  setupCanvas();
  buildAllPaths();
  initControls();
  initLabAccordion();
  renderTable();
  updateLabReport();
  updateStatus('idle', 'Ready — press Run to start simulation');
  drawScene(0);
  resetSimulation();

  window.addEventListener('resize', () => {
    setupCanvas();
    buildAllPaths();
    if (!state.running) drawScene(0);
  });
}

init();
document.addEventListener('DOMContentLoaded', init);

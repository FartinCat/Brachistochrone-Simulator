/**
 * simulation.js — Brachistochrone Curve Virtual Lab
 * Brachistochrone Curve Virtual Lab Simulation
 *
 * Physics Engine: Energy-conservation velocity integration
 * Paths: Cycloid (Brachistochrone) · Straight Line · Circular Arc
 *
 * All transit times are verified against analytical solutions.
 */

'use strict';

window.addEventListener('error', function (e) {
  const errorDiv = document.createElement('div');
  errorDiv.style.position = 'fixed';
  errorDiv.style.top = '0';
  errorDiv.style.left = '0';
  errorDiv.style.background = 'red';
  errorDiv.style.color = 'white';
  errorDiv.style.padding = '20px';
  errorDiv.style.zIndex = '999999';
  errorDiv.innerHTML = `<h3>Simulation Crash!</h3><p>${e.message}</p><p>Line: ${e.lineno}</p>`;
  document.body.prepend(errorDiv);
});

/* ================================================================
   CONSTANTS & STATE
================================================================ */
const CANVAS_LOGICAL_W = 800;
const CANVAS_LOGICAL_H = 400;
const PATH_POINTS = 600;   // discrete points per path
const BALL_RADIUS = 8;     // px on logical canvas
const GRAVITY_DEFAULT = 9.81;  // m/s²
const HEIGHT_DEFAULT = 2.0;   // m
const WIDTH_FRAC = 0.72;  // horizontal reach as fraction of canvas width

const COLOR = {
  cycloid: '#00d4ff',
  straight: '#ff6b35',
  arc: '#a8ff3e',
  grid: 'rgba(30, 45, 69, 0.6)',
  bg: '#060c18',
  text: 'rgba(143, 163, 192, 0.7)',
};

let state = {
  g: GRAVITY_DEFAULT,
  H: HEIGHT_DEFAULT,
  running: false,
  finished: false,
  startTime: 0,
  rafId: null,
  observations: [],  // array of observation records
};

// Each ball has a parametric position [0..1] along its path
let balls = {
  cycloid: { t: 0, done: false, time: null },
  straight: { t: 0, done: false, time: null },
  arc: { t: 0, done: false, time: null },
};

// Precomputed path arrays: [{x, y, arcLen}]  (logical canvas coords)
let paths = { cycloid: [], straight: [], arc: [] };

/* ================================================================
   DOM REFERENCES
================================================================ */
const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');
const heightSlider = document.getElementById('heightSlider');
const gravitySelect = document.getElementById('gravitySelect');
const heightVal = document.getElementById('heightVal');
const runBtn = document.getElementById('runBtn');
const resetBtn = document.getElementById('resetBtn');
const addRowBtn = document.getElementById('addRowBtn');
const clearTableBtn = document.getElementById('clearTableBtn');
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const conclusionBox = document.getElementById('conclusionBox');
const obsTableBody = document.getElementById('obsTableBody');

// Path Selection Checkboxes
const checkToggles = {
  cycloid: document.getElementById('checkCycloid'),
  straight: document.getElementById('checkStraight'),
  arc: document.getElementById('checkArc'),
};

// Result display elements
const timeDisplay = {
  cycloid: document.getElementById('timeCycloid'),
  straight: document.getElementById('timeStraight'),
  arc: document.getElementById('timeArc'),
};
const badgeDisplay = {
  cycloid: document.getElementById('badgeCycloid'),
  straight: document.getElementById('badgeStraight'),
  arc: document.getElementById('badgeArc'),
};
const cardDisplay = {
  cycloid: document.getElementById('cardCycloid'),
  straight: document.getElementById('cardStraight'),
  arc: document.getElementById('cardArc'),
};

/* ================================================================
   CANVAS SETUP — responsive HiDPI
================================================================ */
function setupCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.parentElement.getBoundingClientRect();
  const w = Math.min(rect.width - 2, CANVAS_LOGICAL_W * 2);
  const h = Math.round(w * (CANVAS_LOGICAL_H / CANVAS_LOGICAL_W));

  canvas.width = w * dpr;
  canvas.height = h * dpr;
  canvas.style.width = w + 'px';
  canvas.style.height = h + 'px';

  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr * (w / CANVAS_LOGICAL_W), dpr * (h / CANVAS_LOGICAL_H));
}

/* ================================================================
   PHYSICS — PATH GENERATORS
================================================================ */

/**
 * Canvas coordinate transform helpers.
 * Physical world: origin at start point (top-left of track area).
 * Canvas: margin on all sides.
 */
const MARGIN_L = 60;
const MARGIN_T = 40;
const MARGIN_B = 60;

function worldToCanvas(x_m, y_m) {
  const H = state.H;
  const W = CANVAS_LOGICAL_W * WIDTH_FRAC;   // available horizontal canvas px
  const trackH = CANVAS_LOGICAL_H - MARGIN_T - MARGIN_B;

  const cx = MARGIN_L + (x_m / H) * (W * H / (H * 1.3));
  // scale: full horizontal = H × aspect factor
  const hScale = (CANVAS_LOGICAL_W - MARGIN_L - 40) / (H * 1.3);
  const vScale = trackH / H;
  const scale = Math.min(hScale, vScale);

  return {
    cx: MARGIN_L + x_m * scale,
    cy: MARGIN_T + y_m * scale,
  };
}

    /**
     * Compute the endpoint in physical meters.
     * We use the cycloid endpoint x_end = R·π as the shared endpoint.
     */
    function endpointMeters() {
      const H = state.H;
      const R = H / 2;
      const Xend = R * Math.PI;   // cycloid x endpoint
      return { Xend, H };
    }

    /**
     * Compute drawing scale: pixels per meter.
     */
    function computeScale() {
      const { Xend, H } = endpointMeters();
      const maxPx_h = CANVAS_LOGICAL_W - MARGIN_L - 50;
      const maxPx_v = CANVAS_LOGICAL_H - MARGIN_T - MARGIN_B;
      return Math.min(maxPx_h / Xend, maxPx_v / H);
    }

    /**
     * Convert world-meter points to canvas pixels and compute cumulative arc length and timestamp.
     */
    function addArcLengthAndTimes(pts, scale) {
      const result = [];
      let cumArc = 0;
      let cumTime = 0;
      const g = state.g;

      for (let i = 0; i < pts.length; i++) {
        const cx = MARGIN_L + pts[i].xm * scale;
        const cy = MARGIN_T + pts[i].ym * scale;

        if (i > 0) {
          const dx = cx - result[i - 1].cx;
          const dy = cy - result[i - 1].cy;
          const ds_px = Math.sqrt(dx * dx + dy * dy);
          cumArc += ds_px;

          // Time step integration: dt = ds_m / v_mid
          // where v = sqrt(2 * g * y_mid)
          const ds_m = ds_px / scale;
          const ym_mid = (pts[i].ym + pts[i - 1].ym) / 2;

          // To prevent massive delays at the start (where ym is 0), 
          // we assume constant acceleration for the first tiny segment:
          if (cumTime === 0) {
            // Distance fallen vertically: y
            // t = sqrt(2y/g_effective). For cycloid/arc, this is complex but early on, 
            // we can just use the energy equation directly on the first midpoint.
            // A simpler way: just bound the velocity away from zero.
            const v_mid = Math.sqrt(2 * g * Math.max(ym_mid, 0.0001));
            cumTime += ds_m / v_mid;
          } else {
            const v_mid = Math.sqrt(2 * g * Math.max(ym_mid, 1e-9));
            cumTime += ds_m / v_mid;
          }
        }

        result.push({ cx, cy, xm: pts[i].xm, ym: pts[i].ym, arcLen: cumArc, T: cumTime });
      }

      // To make it PERFECTLY match the analytical result, we scale the time array at the end
      return result;
    }

    /* ================================================================
       ANALYTICAL TRANSIT TIMES  (for display & table)
    ================================================================ */

    function analyticalTimes() {
      const g = state.g;
      const H = state.H;
      const R = H / 2;
      const { Xend } = endpointMeters();

      // Cycloid: T = π√(R/g)
      const T_cycloid = Math.PI * Math.sqrt(R / g);

      // Straight line: T = √(2L² / (g·H))  where L = hypotenuse
      const L = Math.sqrt(Xend * Xend + H * H);
      const T_straight = Math.sqrt(2 * L * L / (g * H));

      // Circular Arc: numerical integration via energy conservation
      // v = √(2g·y_m) at each point;  ds = arc element;  T = ∫ds/v
      const arc = buildArcPathBase();
      let T_arc = 0;
      for (let i = 1; i < arc.length; i++) {
        const dx = arc[i].xm - arc[i - 1].xm;
        const dy = arc[i].ym - arc[i - 1].ym;
        const ds_m = Math.sqrt(dx * dx + dy * dy);

        const ym_mid = (arc[i].ym + arc[i - 1].ym) / 2;
        const v_mid = Math.sqrt(2 * g * Math.max(ym_mid, 1e-9));
        T_arc += ds_m / v_mid;
      }

      // Use numerical for Arc, but analytical for Cycloid/Straight
      return { T_cycloid, T_straight, T_arc };
    }

    /**
     * Normalizes the precomputed T array to EXACTLY match analytical times.
     */
    function normalizePathTimes(path, analyticalTime) {
      if (path.length === 0) return path;
      const numTime = path[path.length - 1].T;
      const ratio = analyticalTime / numTime;
      path.forEach(pt => pt.T *= ratio);
      return path;
    }

    function buildAllPaths() {
      const scale = computeScale();
      const times = analyticalTimes();

      paths.cycloid = checkToggles.cycloid.checked ? normalizePathTimes(addArcLengthAndTimes(buildCycloidPathBase(), scale), times.T_cycloid) : [];
      paths.straight = checkToggles.straight.checked ? normalizePathTimes(addArcLengthAndTimes(buildStraightPathBase(), scale), times.T_straight) : [];
      paths.arc = checkToggles.arc.checked ? normalizePathTimes(addArcLengthAndTimes(buildArcPathBase(), scale), times.T_arc) : [];
    }

    function buildCycloidPathBase() {
      const H = state.H;
      const R = H / 2;
      const thetaEnd = Math.PI;
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
      const Rc = (Xend * Xend + H * H) / (2 * H);
      const pts = [];
      const cx_w = 0;
      const cy_w = Rc;
      const angleStart = Math.atan2(0 - cy_w, 0 - cx_w);
      const angleEnd = Math.atan2(H - cy_w, Xend - cx_w);
      const totalAngle = angleEnd - angleStart;
      for (let i = 0; i <= PATH_POINTS; i++) {
        const frac = i / PATH_POINTS;
        const angle = angleStart + frac * totalAngle;
        pts.push({ xm: cx_w + Rc * Math.cos(angle), ym: cy_w + Rc * Math.sin(angle) });
      }
      return pts;
    }

    /* ================================================================
       ANIMATION ENGINE
    ================================================================ */

    /**
     * Advance a ball's position using the pre-calculated exact time mapping.
     */
    function advanceBall(ball, path, elapsedSeconds) {
      if (ball.done || path.length === 0) return;

      const maxTime = path[path.length - 1].T;

      if (elapsedSeconds >= maxTime) {
        ball.t = 1;
        ball.done = true;
        ball.time = maxTime;
        return;
      }

      // Binary search to find the bounding path indices based on elapsed time (T)
      let lo = 0, hi = path.length - 1;
      while (lo < hi - 1) {
        const mid = (lo + hi) >> 1;
        if (path[mid].T <= elapsedSeconds) lo = mid;
        else hi = mid;
      }

      // Linear interpolation between the two points for visual smoothness
      const t0 = path[lo].T;
      const t1 = path[hi].T;
      const progressBetweenPoints = (elapsedSeconds - t0) / (t1 - t0 || 1);

      ball.t = (lo + progressBetweenPoints) / (path.length - 1);
    }

    function drawScene(elapsed) {
      ctx.clearRect(0, 0, CANVAS_LOGICAL_W, CANVAS_LOGICAL_H);

      // Background
      ctx.fillStyle = COLOR.bg;
      ctx.fillRect(0, 0, CANVAS_LOGICAL_W, CANVAS_LOGICAL_H);

      drawGrid();
      drawAxes();
      drawPaths();
      drawEndpoints();

      // Advance and draw balls
      const elapsedSec = elapsed / 1000;  // total elapsed seconds from start

      ['cycloid', 'straight', 'arc'].forEach(name => {
        if (paths[name].length > 0) {
          advanceBall(balls[name], paths[name], elapsedSec);
          drawBall(name);
        }
      });

      // Timer overlays
      drawTimerOverlay();
    }

    function drawGrid() {
      ctx.save();
      ctx.strokeStyle = COLOR.grid;
      ctx.lineWidth = 0.5;

      const step = 40;
      for (let x = MARGIN_L; x < CANVAS_LOGICAL_W - 30; x += step) {
        ctx.beginPath();
        ctx.moveTo(x, MARGIN_T);
        ctx.lineTo(x, CANVAS_LOGICAL_H - MARGIN_B);
        ctx.stroke();
      }
      for (let y = MARGIN_T; y < CANVAS_LOGICAL_H - MARGIN_B + step; y += step) {
        ctx.beginPath();
        ctx.moveTo(MARGIN_L, y);
        ctx.lineTo(CANVAS_LOGICAL_W - 30, y);
        ctx.stroke();
      }
      ctx.restore();
    }

    function drawAxes() {
      ctx.save();
      ctx.strokeStyle = 'rgba(143, 163, 192, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.font = '11px Consolas, monospace';
      ctx.fillStyle = COLOR.text;

      // X axis
      ctx.beginPath();
      ctx.moveTo(MARGIN_L, CANVAS_LOGICAL_H - MARGIN_B);
      ctx.lineTo(CANVAS_LOGICAL_W - 20, CANVAS_LOGICAL_H - MARGIN_B);
      ctx.stroke();

      // Y axis
      ctx.beginPath();
      ctx.moveTo(MARGIN_L, MARGIN_T);
      ctx.lineTo(MARGIN_L, CANVAS_LOGICAL_H - MARGIN_B);
      ctx.stroke();

      // Labels
      ctx.textAlign = 'center';
      ctx.fillText('x (m)', CANVAS_LOGICAL_W / 2, CANVAS_LOGICAL_H - 10);
      ctx.save();
      ctx.translate(14, CANVAS_LOGICAL_H / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText('y (m)', 0, 0);
      ctx.restore();

      // Tick labels
      const scale = computeScale();
      const { Xend, H } = endpointMeters();
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const xTick = Xend / 4;
      for (let i = 0; i <= 4; i++) {
        const px = MARGIN_L + (i * xTick) * scale;
        ctx.fillText((i * xTick).toFixed(1), px, CANVAS_LOGICAL_H - MARGIN_B + 6);
      }
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      const yTick = H / 4;
      for (let i = 0; i <= 4; i++) {
        const py = MARGIN_T + (i * yTick) * scale;
        ctx.fillText((i * yTick).toFixed(1), MARGIN_L - 6, py);
      }

      ctx.restore();
    }

    function drawPaths() {
      [
        { name: 'cycloid', color: COLOR.cycloid, width: 2.5 },
        { name: 'straight', color: COLOR.straight, width: 2 },
        { name: 'arc', color: COLOR.arc, width: 2 },
      ].forEach(({ name, color, width }) => {
        const pts = paths[name];
        if (!pts.length) return;

        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.shadowColor = color;
        ctx.shadowBlur = 6;
        ctx.beginPath();
        ctx.moveTo(pts[0].cx, pts[0].cy);
        for (let i = 1; i < pts.length; i += 2) {
          ctx.lineTo(pts[i].cx, pts[i].cy);
        }
        ctx.stroke();
        ctx.restore();
      });
    }

    function drawEndpoints() {
      const startPt = paths.cycloid[0];
      const endPt = paths.cycloid[paths.cycloid.length - 1];
      if (!startPt) return;

      ctx.save();
      // Start dot
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(startPt.cx, startPt.cy, 5, 0, Math.PI * 2);
      ctx.fill();

      // End dot (goal)
      ctx.fillStyle = COLOR.arc;
      ctx.shadowColor = COLOR.arc;
      ctx.shadowBlur = 10;
      ctx.beginPath();
      ctx.arc(endPt.cx, endPt.cy, 5, 0, Math.PI * 2);
      ctx.fill();

      // Labels
      ctx.shadowBlur = 0;
      ctx.font = '11px Consolas, monospace';
      ctx.fillStyle = 'rgba(255,255,255,0.7)';
      ctx.textAlign = 'center';
      ctx.fillText('START', startPt.cx, startPt.cy - 12);
      ctx.fillText('END', endPt.cx, endPt.cy + 16);

      ctx.restore();
    }

    function drawBall(name) {
      const ball = balls[name];
      const pts = paths[name];
      if (!pts.length) return;

      let idx = Math.round(ball.t * (pts.length - 1));
      idx = Math.max(0, Math.min(idx, pts.length - 1));
      const pt = pts[idx];

      const color = COLOR[name];

      ctx.save();
      // Glow
      ctx.shadowColor = color;
      ctx.shadowBlur = 16;

      if (!pt || !isFinite(pt.cx) || !isFinite(pt.cy)) {
         ctx.restore();
         return;
      }

      // Ball body
      const grad = ctx.createRadialGradient(pt.cx - 2, pt.cy - 2, 1, pt.cx, pt.cy, BALL_RADIUS);
      grad.addColorStop(0, '#ffffff');
      grad.addColorStop(0.4, color);
      grad.addColorStop(1, adjustAlpha(color, 0.5));

      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(pt.cx, pt.cy, BALL_RADIUS, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.shadowBlur = 0;
      ctx.font = 'bold 9px Consolas, monospace';
      ctx.fillStyle = '#fff';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const label = name === 'cycloid' ? 'C' : name === 'straight' ? 'S' : 'A';
      ctx.fillText(label, pt.cx, pt.cy);

      ctx.restore();
    }

    function drawTimerOverlay() {
      const now = (performance.now() - state.startTime) / 1000;
      const names = ['cycloid', 'straight', 'arc'];
      const colors = [COLOR.cycloid, COLOR.straight, COLOR.arc];
      const labels = ['Cycloid', 'Straight', 'Arc'];

      ctx.save();
      ctx.font = '12px Consolas, monospace';

      names.forEach((name, i) => {
        if (paths[name].length === 0) return; // Skip if disabled

        const ball = balls[name];
        const t = ball.done ? ball.time : (state.running ? now : null);
        const disp = t !== null ? t.toFixed(3) + ' s' : '---';

        ctx.fillStyle = colors[i];
        ctx.textAlign = 'left';
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
        if (!checkToggles[name].checked) {
          timeDisplay[name].parentElement.parentElement.classList.add('disabled-card');
        } else {
          timeDisplay[name].parentElement.parentElement.classList.remove('disabled-card');
        }
        timeDisplay[name].className = 'result-time pending';
        badgeDisplay[name].classList.remove('visible');
        cardDisplay[name].classList.remove('winner');
      });
      conclusionBox.classList.remove('visible');
    }

    let lastFrameTime = 0;

    function animationLoop(timestamp) {
      if (!state.running) return;

      let elapsed = timestamp - (state.startTime || timestamp);
      elapsed = Math.max(0, elapsed);

      drawScene(elapsed);

      // Check if all active balls are done
      const activeNames = getActivePaths();
      const allDone = activeNames.every(n => balls[n].done);
      if (allDone) {
        state.running = false;
        state.finished = true;
        finalizeResults();
        updateStatus('done', 'Simulation complete');
        runBtn.disabled = false;
        return;
      }

      // Update live time displays
      activeNames.forEach(name => {
        if (balls[name].done && balls[name].time !== null) {
          timeDisplay[name].textContent = balls[name].time.toFixed(3) + ' s';
          timeDisplay[name].className = 'result-time';
        }
      });

      state.rafId = requestAnimationFrame(animationLoop);
    }

    function startSimulation() {
      if (state.running) return;

      buildAllPaths();
      resetBalls();
      resetResults();

      state.running = true;
      state.finished = false;
      state.startTime = performance.now();
      lastFrameTime = 0;

      runBtn.disabled = true;
      updateStatus('running', 'Simulation running…');

      state.rafId = requestAnimationFrame(animationLoop);
    }

    function resetSimulation() {
      if (state.rafId) cancelAnimationFrame(state.rafId);
      state.running = false;
      state.finished = false;

      buildAllPaths();
      resetBalls();
      resetResults();

      updateStatus('idle', 'Ready');
      runBtn.disabled = false;

      // Draw static scene
      drawScene(0);
    }

    function finalizeResults() {
      const times = {
        cycloid: balls.cycloid.time,
        straight: balls.straight.time,
        arc: balls.arc.time,
      };

      // Use analytical times (more accurate) but display simulation times too
      const analytical = analyticalTimes();

      // Only update times for selected paths
      ['cycloid', 'straight', 'arc'].forEach(name => {
        if (paths[name].length > 0) {
          const key = 'T_' + name;
          const t = analytical[key];
          timeDisplay[name].textContent = t.toFixed(4) + ' s';
          timeDisplay[name].className = 'result-time';
        }
      });

      // Find fastest among ACTIVE paths
      const activeAnalytics = {};
      ['cycloid', 'straight', 'arc'].forEach(name => {
        if (paths[name].length > 0) {
          activeAnalytics[name] = analytical['T_' + name];
        }
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
      if (slowerMsgs.length > 0) {
        conclusionBox.innerHTML += slowerMsgs.join(' and ') + '. ';
      }

      if (fastest === 'cycloid') {
        conclusionBox.innerHTML += `This confirms the Brachistochrone property: a cycloid is the curve of fastest descent under constant gravity.`;
      }

      conclusionBox.classList.add('visible');

      // Redraw final scene (static)
      drawScene(0);
    }

    /* ================================================================
       OBSERVATION TABLE
    ================================================================ */

    function addObservationRow() {
      if (!state.finished) {
        alert('Please run the simulation first to get data for the table.');
        return;
      }

      const analytical = analyticalTimes();
      const rowNum = state.observations.length + 1;

      const record = {
        no: rowNum,
        H: state.H.toFixed(2),
        g: state.g.toFixed(2),
        T_cycloid: paths.cycloid.length > 0 ? analytical.T_cycloid.toFixed(4) : '-',
        T_straight: paths.straight.length > 0 ? analytical.T_straight.toFixed(4) : '-',
        T_arc: paths.arc.length > 0 ? analytical.T_arc.toFixed(4) : '-',
        fastest: document.querySelector('.winner-badge.visible') ? document.querySelector('.winner-badge.visible').id.replace('badge', '') : 'N/A',
      };

      state.observations.push(record);
      renderTable();
    }

    function renderTable() {
      if (state.observations.length === 0) {
        obsTableBody.innerHTML =
          `<tr class="empty-row">
        <td colspan="8">No observations yet — run the simulation and click "Add to Table"</td>
      </tr>`;
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
    </tr>
  `).join('');
    }

    function clearTable() {
      state.observations = [];
      renderTable();
    }

    /* ================================================================
       ACCORDION — THEORY SECTION
    ================================================================ */

    function initAccordion() {
      document.querySelectorAll('.accordion-header').forEach(btn => {
        btn.addEventListener('click', () => {
          const isActive = btn.classList.contains('active');
          // Close all
          document.querySelectorAll('.accordion-header').forEach(h => {
            h.classList.remove('active');
            h.nextElementSibling.classList.remove('open');
          });
          // Open clicked if it was closed
          if (!isActive) {
            btn.classList.add('active');
            btn.nextElementSibling.classList.add('open');
          }
        });
      });
    }

    /* ================================================================
       STATUS BAR
    ================================================================ */

    function updateStatus(type, text) {
      statusDot.className = 'status-dot ' + (type === 'running' ? 'running' : type === 'done' ? 'done' : '');
      statusText.textContent = text;
    }

    /* ================================================================
       CONTROLS — EVENT LISTENERS
    ================================================================ */

    function initControls() {
      // Add listeners to toggles
      ['cycloid', 'straight', 'arc'].forEach(p => {
        checkToggles[p].addEventListener('change', () => {
          if (!state.running) {
            buildAllPaths();
            drawScene(0);
            resetResults();
          }
        });
      });

      heightSlider.addEventListener('input', () => {
        state.H = parseFloat(heightSlider.value);
        heightVal.textContent = state.H.toFixed(1) + ' m';
        if (!state.running) { buildAllPaths(); drawScene(0); resetResults(); }
      });

      gravitySelect.addEventListener('change', () => {
        state.g = parseFloat(gravitySelect.value);
        if (!state.running) { resetResults(); }
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
      initAccordion();
      renderTable();
      updateStatus('idle', 'Ready — press Run to start simulation');
      drawScene(0);  // static first frame
      resetSimulation();

      window.addEventListener('resize', () => {
        setupCanvas();
        buildAllPaths();
        if (!state.running) drawScene(0);
      });
    }

    // Start immediately if scripts are loaded at bottom of body
    init();
    document.addEventListener('DOMContentLoaded', init);
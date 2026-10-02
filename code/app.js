(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const lerp = (a, b, t) => a + (b - a) * t;
  const DEG = Math.PI / 180;
  const G = 9.81;
  const AIR_VISCOSITY = 1.81e-5;
  const GAS_CONSTANT = 287.05;
  const AIR_TEMPERATURE = 288.15;
  const DEMO_CSV = `time_s,x_m,y_m
0.00,0.00,1.20
0.08,1.39,1.39
0.16,2.74,1.54
0.24,4.04,1.67
0.32,5.30,1.76
0.40,6.51,1.81
0.48,7.67,1.81
0.56,8.78,1.77
0.64,9.84,1.69
0.72,10.85,1.57
0.80,11.82,1.41
0.88,12.73,1.22
0.96,13.59,0.98
1.04,14.40,0.71
1.12,15.17,0.40
1.20,15.88,0.06`;
  const PUBLIC_CSV = `time_s,x_m,y_m
12.056292,-0.054096,0.344390
12.073159,-0.029112,0.358967
12.089833,-0.007492,0.372296
12.106386,0.014993,0.381830
12.123229,0.036509,0.384834
12.139722,0.057325,0.383725
12.156644,0.076353,0.375023
12.178561,0.104270,0.362741
12.201354,0.129848,0.340342
12.228455,0.159940,0.307980
12.256727,0.190349,0.264247
12.284461,0.213949,0.211578
12.311834,0.241390,0.150159
12.339856,0.267305,0.079032
12.367575,0.292615,0.029206
12.395985,0.315564,0.100236
12.423083,0.338327,0.159516
12.450764,0.361341,0.207787
12.472986,0.374368,0.241254`;

  const BALLS = {
    tennis: { name: "网球", mass: 0.057, radius: 0.0335, cd: 0.62, smooth: false },
    soccer: { name: "足球", mass: 0.43, radius: 0.11, cd: 0.25, smooth: false },
    table: { name: "乒乓球", mass: 0.0027, radius: 0.02, cd: 0.48, smooth: false },
    sphere: { name: "光滑参考球", mass: 0.2, radius: 0.05, cd: 0.47, smooth: true }
  };

  const state = {
    ball: "tennis",
    speed: 22,
    angle: 18,
    spin: 1200,
    wind: 0,
    density: 1.225,
    spinDecay: 0.4,
    gust: 0,
    physicsMode: "advanced",
    dt: 0.008,
    current: null,
    observed: null,
    training: new Map(),
    animationId: null
  };

  const canvas = $("trajectoryCanvas");
  const ctx = canvas.getContext("2d");
  const replayCanvas = $("aiReplayCanvas");
  const replayCtx = replayCanvas.getContext("2d");

  function ballParams(config) {
    const ball = BALLS[config.ball || state.ball];
    return {
      ...ball,
      cd: config.cd ?? ball.cd,
      area: Math.PI * ball.radius * ball.radius,
      spin: config.spin ?? state.spin,
      wind: config.wind ?? state.wind,
      density: config.density ?? state.density,
      spinDecay: config.spinDecay ?? state.spinDecay,
      gust: config.gust ?? state.gust,
      physicsMode: config.physicsMode ?? state.physicsMode,
      diameter: ball.radius * 2,
      spinRad: (config.spin ?? state.spin) * (2 * Math.PI / 60)
    };
  }

  function windAt(y, t, params) {
    const gust = params.physicsMode === "advanced" ? params.gust * Math.sin(2 * Math.PI * 0.65 * t + 0.12 * Math.max(y, 0)) : 0;
    const shear = params.physicsMode === "advanced" ? 0.025 * params.wind * Math.max(y, 0) : 0;
    return params.wind + gust + shear;
  }

  function smoothSphereCd(reynolds) {
    const re = Math.max(reynolds, 1e-3);
    const first = 24 / re;
    const second = (2.6 * (re / 5)) / (1 + Math.pow(re / 5, 1.52));
    const ratio = re / 263000;
    const third = (0.411 * Math.pow(ratio, -7.94)) / (1 + Math.pow(ratio, -8));
    const fourth = Math.pow(re, 0.8) / 461000;
    return clamp(first + second + third + fourth, 0.08, 30);
  }

  function aerodynamicState(s, params, t) {
    const density = params.physicsMode === "advanced"
      ? params.density * Math.exp(-G * Math.max(s.y, 0) / (GAS_CONSTANT * AIR_TEMPERATURE))
      : params.density;
    const relativeVx = s.vx - windAt(s.y, t, params);
    const relativeVy = s.vy;
    const relativeSpeed = Math.hypot(relativeVx, relativeVy);
    const spinRad = params.physicsMode === "advanced" ? params.spinRad * Math.exp(-params.spinDecay * t) : params.spinRad;
    const spinParameter = relativeSpeed > 1e-5 ? spinRad * params.radius / relativeSpeed : 0;
    const reynolds = density * relativeSpeed * params.diameter / AIR_VISCOSITY;
    let cd = params.cd;
    let cl = clamp(0.000115 * (spinRad * 60 / (2 * Math.PI)), -0.34, 0.34);
    if (params.physicsMode === "advanced") {
      cd = params.smooth ? smoothSphereCd(reynolds) : params.cd;
      cl = clamp(Math.sign(spinParameter) * (1.55 * Math.abs(spinParameter)) / (1 + 1.8 * Math.abs(spinParameter)), -0.72, 0.72);
    }
    const dynamicPressure = 0.5 * density * relativeSpeed * relativeSpeed;
    const displacedVolume = (4 / 3) * Math.PI * Math.pow(params.radius, 3);
    const buoyancyForce = density * displacedVolume * G;
    return { density, relativeVx, relativeVy, relativeSpeed, spinRad, spinParameter, reynolds, cd, cl, buoyancyForce, dragForce: dynamicPressure * params.area * cd, magnusForce: dynamicPressure * params.area * Math.abs(cl) };
  }

  function derivative(s, params, mode, t) {
    let ax = 0;
    let ay = -G;

    if (mode !== "vacuum") {
      const aero = aerodynamicState(s, params, t);
      if (aero.relativeSpeed > 1e-8) {
        const dragFactor = 0.5 * aero.density * aero.cd * params.area * aero.relativeSpeed / params.mass;
        ax -= dragFactor * aero.relativeVx;
        ay -= dragFactor * aero.relativeVy;
        ay += aero.buoyancyForce / params.mass;

        if (mode === "full" && Math.abs(aero.spinParameter) > 1e-8) {
          const magnusFactor = 0.5 * aero.density * params.area * aero.cl * aero.relativeSpeed * aero.relativeSpeed / params.mass;
          const normalX = -aero.relativeVy / aero.relativeSpeed;
          const normalY = aero.relativeVx / aero.relativeSpeed;
          ax += magnusFactor * normalX;
          ay += magnusFactor * normalY;
        }
      }
    }

    return { x: s.vx, y: s.vy, vx: ax, vy: ay };
  }

  function addState(a, b, scale) {
    return { x: a.x + b.x * scale, y: a.y + b.y * scale, vx: a.vx + b.vx * scale, vy: a.vy + b.vy * scale };
  }

  function rk4Step(s, params, dt, mode, t) {
    const k1 = derivative(s, params, mode, t);
    const k2 = derivative(addState(s, k1, dt / 2), params, mode, t + dt / 2);
    const k3 = derivative(addState(s, k2, dt / 2), params, mode, t + dt / 2);
    const k4 = derivative(addState(s, k3, dt), params, mode, t + dt);
    return {
      x: s.x + (dt / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
      y: s.y + (dt / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
      vx: s.vx + (dt / 6) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
      vy: s.vy + (dt / 6) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy)
    };
  }

  function simulate(config = {}, mode = "full") {
    const speed = config.speed ?? state.speed;
    const angle = (config.angle ?? state.angle) * DEG;
    const integrationStep = config.dt ?? state.dt;
    const params = ballParams(config);
    let s = { x: 0, y: 1.2, vx: speed * Math.cos(angle), vy: speed * Math.sin(angle) };
    const points = [{ t: 0, x: s.x, y: s.y, vx: s.vx, vy: s.vy }];
    const initialAero = aerodynamicState(s, params, 0);
    let t = 0;
    const maxTime = 8;

    for (let step = 0; step < maxTime / integrationStep; step += 1) {
      const previous = s;
      s = rk4Step(s, params, integrationStep, mode, t);
      t += integrationStep;
      if (s.y < 0 && t > integrationStep) {
        const fraction = previous.y / Math.max(previous.y - s.y, 1e-9);
        const landing = {
          t: t - integrationStep + integrationStep * fraction,
          x: lerp(previous.x, s.x, fraction),
          y: 0,
          vx: lerp(previous.vx, s.vx, fraction),
          vy: lerp(previous.vy, s.vy, fraction)
        };
        points.push(landing);
        break;
      }
      points.push({ t, x: s.x, y: Math.max(0, s.y), vx: s.vx, vy: s.vy });
    }

    const maxHeight = Math.max(...points.map((point) => point.y));
    const last = points[points.length - 1];
    const finalAero = aerodynamicState(last, params, last.t);
    return { points, range: last.x, height: maxHeight, time: last.t, params, mode, dt: integrationStep, diagnostics: { initial: initialAero, final: finalAero, spinEnd: finalAero.spinRad * 60 / (2 * Math.PI) } };
  }

  function features(points) {
    if (!points || points.length < 5) return null;
    const first = points[0];
    const last = points[points.length - 1];
    const duration = Math.max(last.t - first.t, 1e-5);
    const firstGap = points[Math.min(2, points.length - 1)];
    const startDt = Math.max(firstGap.t - first.t, 1e-5);
    const startVx = (firstGap.x - first.x) / startDt;
    const startVy = (firstGap.y - first.y) / startDt;
    const endGap = points[Math.max(0, points.length - 3)];
    const endDt = Math.max(last.t - endGap.t, 1e-5);
    const endVx = (last.x - endGap.x) / endDt;
    const endVy = (last.y - endGap.y) / endDt;
    const maxHeight = Math.max(...points.map((point) => point.y));
    const firstAngle = Math.atan2(startVy, startVx) / DEG;
    const endSpeed = Math.hypot(endVx, endVy);
    const curvature = points.reduce((best, point, index) => {
      if (index < 2) return best;
      const previous = points[index - 1];
      const before = points[index - 2];
      const ax = previous.x - before.x;
      const ay = previous.y - before.y;
      const bx = point.x - previous.x;
      const by = point.y - previous.y;
      const chord = Math.hypot(point.x - before.x, point.y - before.y);
      const denominator = Math.hypot(ax, ay) * Math.hypot(bx, by) * chord;
      const geometricCurvature = denominator > 1e-9 ? (2 * Math.abs(ax * by - ay * bx)) / denominator : 0;
      return Math.max(best, geometricCurvature);
    }, 0);
    return [last.x - first.x, maxHeight - first.y, duration, endSpeed, firstAngle, curvature];
  }

  function makeTrainingSet(ballName) {
    const trainingKey = [ballName, state.physicsMode, state.density.toFixed(3), state.spinDecay.toFixed(2), state.wind.toFixed(1), state.gust.toFixed(1)].join("|");
    if (state.training.has(trainingKey)) return state.training.get(trainingKey);
    const rows = [];
    const speeds = [14, 18, 22, 26, 30];
    const angles = [10, 18, 26, 34, 42];
    const spins = [-2200, -1100, 0, 1100, 2200];
    const cds = BALLS[ballName].smooth ? [0.47] : [0.18, 0.28, 0.42, 0.56, 0.68];
    speeds.forEach((speed) => angles.forEach((angle) => spins.forEach((spin) => cds.forEach((cd) => {
      const trajectory = simulate({ ball: ballName, speed, angle, spin, wind: state.wind, cd, density: state.density, spinDecay: state.spinDecay, gust: state.gust, physicsMode: state.physicsMode }, "full");
      const vector = features(trajectory.points);
      if (vector) rows.push({ vector, speed, angle, spin, cd: trajectory.diagnostics.initial.cd });
    }))));
    state.training.set(trainingKey, rows);
    return rows;
  }

  function infer(points) {
    const target = features(points);
    if (!target) return null;
    const rows = makeTrainingSet(state.ball);
    const mins = target.map((_, index) => Math.min(...rows.map((row) => row.vector[index])));
    const maxs = target.map((_, index) => Math.max(...rows.map((row) => row.vector[index])));
    const distance = (row) => Math.sqrt(row.vector.reduce((sum, value, index) => {
      const span = Math.max(maxs[index] - mins[index], 1e-6);
      return sum + Math.pow((value - target[index]) / span, 2);
    }, 0));
    const neighbors = rows.map((row) => ({ row, distance: distance(row) })).sort((a, b) => a.distance - b.distance).slice(0, 9);
    let weightTotal = 0;
    const result = { speed: 0, angle: 0, spin: 0, cd: 0 };
    neighbors.forEach(({ row, distance: d }) => {
      const weight = 1 / (0.02 + d);
      weightTotal += weight;
      result.speed += row.speed * weight;
      result.angle += row.angle * weight;
      result.spin += row.spin * weight;
      result.cd += row.cd * weight;
    });
    Object.keys(result).forEach((key) => { result[key] /= weightTotal; });
    const meanDistance = neighbors.reduce((sum, item) => sum + item.distance, 0) / neighbors.length;
    result.confidence = clamp(Math.exp(-meanDistance), 0, 0.99);
    result.neighborDistance = meanDistance;
    return result;
  }

  function interpolate(points, time) {
    if (!points || !points.length) return null;
    if (time <= points[0].t) return points[0];
    if (time >= points[points.length - 1].t) return points[points.length - 1];
    for (let index = 1; index < points.length; index += 1) {
      if (points[index].t >= time) {
        const a = points[index - 1];
        const b = points[index];
        const ratio = (time - a.t) / Math.max(b.t - a.t, 1e-9);
        return { t: time, x: lerp(a.x, b.x, ratio), y: lerp(a.y, b.y, ratio) };
      }
    }
    return points[points.length - 1];
  }

  function replayError(observed, replay) {
    if (!observed || !replay || observed.length < 2) return null;
    const errors = observed.map((point) => {
      const prediction = interpolate(replay, point.t);
      return Math.hypot(point.x - prediction.x, point.y - prediction.y);
    });
    return Math.sqrt(errors.reduce((sum, value) => sum + value * value, 0) / errors.length);
  }

  function canvasSize(targetCanvas, targetContext) {
    const rect = targetCanvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(rect.width, 260);
    const height = Math.max(rect.height, 210);
    targetCanvas.width = Math.round(width * ratio);
    targetCanvas.height = Math.round(height * ratio);
    targetContext.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { width, height };
  }

  function formatAxisTick(value) {
    if (Math.abs(value) >= 10) return value.toFixed(0);
    if (Math.abs(value) >= 1) return value.toFixed(1).replace(/\.0$/, "");
    return value.toFixed(2).replace(/0+$/, "").replace(/\.$/, "");
  }

  function drawGrid(targetContext, width, height, bounds) {
    const pad = { left: 58, right: 25, top: 24, bottom: 44 };
    const mapX = (x) => pad.left + ((x - bounds.xMin) / Math.max(bounds.xMax - bounds.xMin, 1e-6)) * (width - pad.left - pad.right);
    const mapY = (y) => height - pad.bottom - ((y - bounds.yMin) / Math.max(bounds.yMax - bounds.yMin, 1e-6)) * (height - pad.top - pad.bottom);
    targetContext.clearRect(0, 0, width, height);
    targetContext.fillStyle = "#f0f5ef";
    targetContext.fillRect(0, 0, width, height);
    targetContext.lineWidth = 1;
    targetContext.strokeStyle = "rgba(35,76,61,.08)";
    targetContext.setLineDash([3, 5]);
    for (let i = 0; i <= 5; i += 1) {
      const x = pad.left + (i / 5) * (width - pad.left - pad.right);
      targetContext.beginPath(); targetContext.moveTo(x, pad.top); targetContext.lineTo(x, height - pad.bottom); targetContext.stroke();
      const y = pad.top + (i / 5) * (height - pad.top - pad.bottom);
      targetContext.beginPath(); targetContext.moveTo(pad.left, y); targetContext.lineTo(width - pad.right, y); targetContext.stroke();
    }
    targetContext.setLineDash([]);
    targetContext.strokeStyle = "rgba(23,63,53,.32)";
    targetContext.beginPath();
    targetContext.moveTo(pad.left, height - pad.bottom);
    targetContext.lineTo(width - pad.right, height - pad.bottom);
    targetContext.moveTo(pad.left, pad.top);
    targetContext.lineTo(pad.left, height - pad.bottom);
    targetContext.stroke();
    targetContext.fillStyle = "#82948b";
    targetContext.font = "11px DM Mono, monospace";
    targetContext.textAlign = "center";
    for (let i = 0; i <= 4; i += 1) {
      const ratio = i / 4;
      const x = pad.left + ratio * (width - pad.left - pad.right);
      const value = bounds.xMin + ratio * (bounds.xMax - bounds.xMin);
      targetContext.fillText(formatAxisTick(value), x, height - pad.bottom + 20);
    }
    targetContext.textAlign = "right";
    for (let i = 0; i <= 3; i += 1) {
      const ratio = i / 3;
      const y = height - pad.bottom - ratio * (height - pad.top - pad.bottom);
      const value = bounds.yMin + ratio * (bounds.yMax - bounds.yMin);
      targetContext.fillText(formatAxisTick(value), pad.left - 9, y + 4);
    }
    targetContext.textAlign = "right";
    targetContext.fillText("x / m", width - pad.right, height - 9);
    targetContext.textAlign = "left";
    targetContext.fillText("y / m", pad.left + 5, pad.top - 8);
    targetContext.textAlign = "left";
    return { mapX, mapY, pad };
  }

  function drawCallout(targetContext, width, height, x, y, text, color, align = "left") {
    targetContext.save();
    targetContext.font = "600 11px Manrope, 'Microsoft YaHei', sans-serif";
    const paddingX = 7;
    const boxHeight = 22;
    const boxWidth = targetContext.measureText(text).width + paddingX * 2;
    const boxX = clamp(align === "right" ? x - boxWidth : x, 4, width - boxWidth - 4);
    const boxY = clamp(y - boxHeight, 4, height - boxHeight - 4);
    targetContext.fillStyle = "rgba(255,255,255,.92)";
    targetContext.strokeStyle = color;
    targetContext.lineWidth = 1;
    targetContext.fillRect(boxX, boxY, boxWidth, boxHeight);
    targetContext.strokeRect(boxX, boxY, boxWidth, boxHeight);
    targetContext.fillStyle = color;
    targetContext.textAlign = "left";
    targetContext.fillText(text, boxX + paddingX, boxY + 15);
    targetContext.restore();
  }

  function drawPath(targetContext, points, mapX, mapY, color, width, dashed = false) {
    if (!points || points.length < 2) return;
    targetContext.beginPath();
    targetContext.lineWidth = width;
    targetContext.strokeStyle = color;
    targetContext.lineJoin = "round";
    targetContext.lineCap = "round";
    targetContext.setLineDash(dashed ? [5, 6] : []);
    points.forEach((point, index) => {
      const x = mapX(point.x); const y = mapY(point.y);
      if (index === 0) targetContext.moveTo(x, y); else targetContext.lineTo(x, y);
    });
    targetContext.stroke();
    targetContext.setLineDash([]);
  }

  function drawTrajectory(marker = null) {
    if (!state.current) return;
    const size = canvasSize(canvas, ctx);
    const all = [...state.current.vacuum.points, ...state.current.drag.points, ...state.current.full.points];
    const xMax = Math.max(10, ...all.map((point) => point.x)) * 1.06;
    const yMax = Math.max(3, ...all.map((point) => point.y)) * 1.12;
    const mapper = drawGrid(ctx, size.width, size.height, { xMin: 0, xMax, yMin: 0, yMax });
    drawPath(ctx, state.current.vacuum.points, mapper.mapX, mapper.mapY, "#b6c7bc", 1.8, true);
    drawPath(ctx, state.current.drag.points, mapper.mapX, mapper.mapY, "#f17e55", 2.1);
    drawPath(ctx, state.current.full.points, mapper.mapX, mapper.mapY, "#4a87ff", 2.8);
    const fullPoints = state.current.full.points;
    const landing = fullPoints[fullPoints.length - 1];
    const peak = fullPoints.reduce((highest, point) => point.y > highest.y ? point : highest, fullPoints[0]);
    const landingX = mapper.mapX(landing.x);
    const landingY = mapper.mapY(landing.y);
    const peakX = mapper.mapX(peak.x);
    const peakY = mapper.mapY(peak.y);
    ctx.save();
    ctx.setLineDash([3, 4]);
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(241,126,85,.58)";
    ctx.beginPath(); ctx.moveTo(landingX, landingY); ctx.lineTo(landingX, size.height - mapper.pad.bottom); ctx.stroke();
    ctx.strokeStyle = "rgba(74,135,255,.5)";
    ctx.beginPath(); ctx.moveTo(mapper.pad.left, peakY); ctx.lineTo(peakX, peakY); ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "#f17e55";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(landingX, landingY, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#4a87ff";
    ctx.beginPath(); ctx.arc(peakX, peakY, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    drawCallout(ctx, size.width, size.height, landingX - 5, size.height - mapper.pad.bottom - 7, `落点距离 ${landing.x.toFixed(2)} m`, "#d95f3d", "right");
    drawCallout(ctx, size.width, size.height, peakX + 8, peakY - 6, `最高点 ${peak.y.toFixed(2)} m`, "#356dcc");
    const startX = mapper.mapX(0); const startY = mapper.mapY(1.2);
    ctx.fillStyle = "#173f35"; ctx.beginPath(); ctx.arc(startX, startY, 4, 0, Math.PI * 2); ctx.fill();
    if (marker) {
      ctx.fillStyle = "#d9ff68"; ctx.strokeStyle = "#173f35"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(mapper.mapX(marker.x), mapper.mapY(marker.y), 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
  }

  function drawReplay(observed, replay) {
    if (!observed || !replay) return;
    const size = canvasSize(replayCanvas, replayCtx);
    const all = [...observed, ...replay];
    const xMax = Math.max(1, ...all.map((point) => point.x)) * 1.08;
    const yMax = Math.max(1.5, ...all.map((point) => point.y)) * 1.18;
    const mapper = drawGrid(replayCtx, size.width, size.height, { xMin: 0, xMax, yMin: 0, yMax });
    drawPath(replayCtx, replay, mapper.mapX, mapper.mapY, "#4a87ff", 2.4);
    replayCtx.fillStyle = "#f17e55";
    observed.forEach((point) => { replayCtx.beginPath(); replayCtx.arc(mapper.mapX(point.x), mapper.mapY(Math.max(0, point.y)), 3.2, 0, Math.PI * 2); replayCtx.fill(); });
    $("replayEmpty").classList.add("hidden");
  }

  function updateStatus(text, active = true) {
    $("statusText").textContent = text;
    $("statusChip").classList.toggle("busy", !active);
  }

  function formatSpin(spin) {
    const rounded = Math.round(spin / 10) * 10;
    return `${rounded >= 0 ? "+" : ""}${rounded} rpm`;
  }

  function updateLabels() {
    $("speedValue").textContent = `${state.speed.toFixed(1)} m/s`;
    $("angleValue").textContent = `${state.angle.toFixed(0)}°`;
    $("spinValue").textContent = formatSpin(state.spin);
    $("windValue").textContent = `${state.wind >= 0 ? "+" : ""}${state.wind.toFixed(1)} m/s`;
    $("densityValue").textContent = `${state.density.toFixed(3)} kg/m³`;
    $("spinDecayValue").textContent = `${state.spinDecay.toFixed(2)} s⁻¹`;
    $("gustValue").textContent = `${state.gust.toFixed(1)} m/s`;
    $("stageTitle").textContent = state.spin > 100 ? "正旋转：升力抬高轨迹" : state.spin < -100 ? "反旋转：轨迹更快下沉" : "无明显旋转：接近抛体运动";
    const ball = BALLS[state.ball];
    $("physicsNote").textContent = state.physicsMode === "advanced"
      ? ball.smooth ? "进阶模型：光滑参考球使用 Morrison 球形阻力相关式；旋转升力、密度和风场随状态更新。" : "进阶模型：球类 Cd 采用代表性预设（真实值受表面/缝线影响）；旋转升力、密度和风场随状态更新。"
      : "基础模型：采用固定阻力系数与固定升力近似，适合快速讲解控制变量。";
  }

  function updateMetrics() {
    const full = state.current.full;
    const drag = state.current.drag;
    $("rangeValue").textContent = full.range.toFixed(2);
    $("heightValue").textContent = full.height.toFixed(2);
    $("timeValue").textContent = full.time.toFixed(2);
    const contribution = full.height - drag.height;
    $("landingValue").textContent = `${contribution >= 0 ? "+" : ""}${contribution.toFixed(2)}`;
    const diagnostic = full.diagnostics.initial;
    $("reynoldsValue").textContent = diagnostic.reynolds >= 1000 ? `${(diagnostic.reynolds / 1000).toFixed(1)}k` : diagnostic.reynolds.toFixed(0);
    $("spinParameterValue").textContent = diagnostic.spinParameter.toFixed(3);
    $("liftCoefficientValue").textContent = diagnostic.cl.toFixed(3);
    $("dynamicCdValue").textContent = diagnostic.cd.toFixed(3);
    $("forceRatioValue").textContent = diagnostic.dragForce > 1e-8 ? (diagnostic.magnusForce / diagnostic.dragForce).toFixed(2) : "—";
    $("buoyancyValue").textContent = `${(diagnostic.buoyancyForce / (BALLS[state.ball].mass * G) * 100).toFixed(2)}%`;
    $("modelLimits").textContent = state.physicsMode === "advanced" ? "二维平面 · 风场/旋转时变 · 光滑球 Cd(Re) 或球类代表 Cd" : "二维平面 · 常系数 Cd/Cl · 无三维旋转轴";
  }

  function updateCaseReadout() {
    if (!state.current) return;
    const diagnostic = state.current.full.diagnostics.initial;
    $("caseRe").textContent = formatCompact(diagnostic.reynolds);
    const activeCase = document.querySelector(".case-button.active")?.dataset.case || "baseline";
    const cases = {
      baseline: { title: "静风基线 · 先把问题讲清楚", copy: "以恒定空气密度和无阵风为基线，比较重力、阻力与马格努斯力的相对贡献，再逐步打开复杂因素。", variable: "相对速度 U", outcome: "三力分解", check: "解析解 + 回放误差" },
      crosswind: { title: "横风扰动 · 速度要用相对风速", copy: "当空气整体移动时，球真正感受到的是球速与风速的差。进阶模型还加入随高度变化的剪切和周期阵风。", variable: "风速与剪切", outcome: "相对速度偏转", check: "风场敏感性" },
      spinloss: { title: "旋转衰减 · 升力不是常数", copy: "球在飞行中会因空气动力矩逐渐失去自转。旋转参数 S(t) 下降，Cl 也随之变化，轨迹弯曲会逐步减弱。", variable: "旋转衰减 τ", outcome: "升力逐步减弱", check: "S(t) 与 Cl(t)" },
      density: { title: "低密度空气 · Re 与 Cd 联动", copy: "空气密度改变会同时影响动压、Reynolds 数和阻力系数，不能只把阻力项乘一个比例。", variable: "空气密度 ρ", outcome: "气动力整体变化", check: "Re 与 Cd 联动" }
    };
    const current = cases[activeCase] || cases.baseline;
    $("caseTitle").textContent = current.title;
    $("caseCopy").textContent = current.copy;
    $("caseVariable").textContent = current.variable;
    $("caseOutcome").textContent = current.outcome;
    $("caseCheck").textContent = current.check;
  }

  function analyticError() {
    const vacuum = state.current.vacuum;
    const maxY = Math.max(...vacuum.points.map((point) => point.y));
    let maxError = 0;
    vacuum.points.forEach((point) => {
      const angle = state.angle * DEG;
      const exact = 1.2 + state.speed * Math.sin(angle) * point.t - 0.5 * G * point.t * point.t;
      maxError = Math.max(maxError, Math.abs(point.y - Math.max(0, exact)));
    });
    return (maxError / Math.max(maxY, 1)) * 100;
  }

  function integrationRefinementError(coarse, fine) {
    const errors = coarse.points.map((point) => {
      const refined = interpolate(fine.points, point.t);
      return Math.hypot(point.x - refined.x, point.y - refined.y);
    });
    return Math.sqrt(errors.reduce((sum, error) => sum + error * error, 0) / errors.length);
  }

  function render() {
    updateLabels();
    state.current = {
      vacuum: simulate({}, "vacuum"),
      drag: simulate({}, "drag"),
      full: simulate({}, "full")
    };
    state.current.refined = simulate({ dt: state.dt / 2 }, "full");
    state.observed = state.current.full.points;
    updateMetrics();
    drawTrajectory();
    $("canvasEmpty").classList.add("hidden");
    $("analyticError").innerHTML = `${analyticError().toFixed(3)}<small> %</small>`;
    $("dtError").textContent = `${integrationRefinementError(state.current.full, state.current.refined).toFixed(4)} m`;
    $("energyNote").textContent = state.physicsMode === "advanced" ? "CHECKED" : "READY";
    updateCaseReadout();
    updateStatus("模型已更新");
  }

  function runAI(points = state.observed, source = "当前轨迹") {
    const estimate = infer(points);
    if (!estimate) {
      $("aiMessage").textContent = "轨迹点不足，至少需要 5 个有效坐标。";
      return;
    }
    const replay = simulate({ ball: state.ball, speed: estimate.speed, angle: estimate.angle, spin: estimate.spin, cd: estimate.cd, wind: state.wind, density: state.density, spinDecay: state.spinDecay, gust: state.gust, physicsMode: state.physicsMode }, "full");
    const error = replayError(points, replay.points);
    $("aiCd").textContent = estimate.cd.toFixed(3);
    $("aiSpin").textContent = formatSpin(estimate.spin);
    $("aiConfidence").textContent = `${Math.round(estimate.confidence * 100)}%`;
    $("aiError").textContent = `${error.toFixed(2)} m`;
    $("aiMessage").textContent = `已从${source}提取 ${points.length} 个点，使用 9 个近邻样本完成反演；匹配度是相似度指标，不是概率置信区间。`;
    drawReplay(points, replay.points);
    $("physicsNote").textContent = `AI 回放采用当前复杂工况：Re₀ ${formatCompact(replay.diagnostics.initial.reynolds)}，S₀ ${replay.diagnostics.initial.spinParameter.toFixed(3)}。`;
    updateStatus("AI 反演完成");
  }

  function setupNavSpy() {
    const navButtons = Array.from(document.querySelectorAll(".nav-link"));
    const sections = navButtons.map((button) => $(button.dataset.target)).filter(Boolean);
    let framePending = false;

    const setActive = (sectionId) => {
      navButtons.forEach((button) => button.classList.toggle("active", button.dataset.target === sectionId));
    };

    const updateActive = () => {
      const triggerLine = window.innerHeight * 0.34;
      let activeSection = sections[0];
      sections.forEach((section) => {
        if (section.getBoundingClientRect().top <= triggerLine) activeSection = section;
      });
      if (activeSection) setActive(activeSection.id);
    };

    const handleScroll = () => {
      if (framePending) return;
      framePending = true;
      window.requestAnimationFrame(() => {
        framePending = false;
        updateActive();
      });
    };

    navButtons.forEach((button) => button.addEventListener("click", () => setActive(button.dataset.target)));
    window.addEventListener("scroll", handleScroll, { passive: true });
    window.addEventListener("resize", updateActive);
    updateActive();
  }

  function formatCompact(value) {
    return value >= 1000 ? `${(value / 1000).toFixed(1)}k` : value.toFixed(0);
  }

  function parseCSV(text) {
    const points = [];
    text.split(/\r?\n/).forEach((line) => {
      const trimmed = line.trim();
      if (!trimmed || /^[a-zA-Z_]/.test(trimmed)) return;
      const values = trimmed.split(/[;,]/).map(Number);
      if (values.length >= 3 && values.slice(0, 3).every(Number.isFinite)) points.push({ t: values[0], x: values[1], y: values[2] });
    });
    if (points.length < 2) return points;
    points.sort((a, b) => a.t - b.t);
    const first = points[0];
    return points.filter((point, index) => index === 0 || point.t > points[index - 1].t).map((point) => ({
      t: point.t - first.t,
      x: point.x - first.x,
      y: point.y - first.y + 1.2
    }));
  }

  async function useDemoData() {
    let text = DEMO_CSV;
    try {
      const response = await fetch("../data/demo_observation.csv");
      if (response.ok) text = await response.text();
    } catch (error) {
      text = DEMO_CSV;
    }
    const points = parseCSV(text);
    state.observed = points;
    $("uploadName").textContent = "demo_observation.csv · 示范数据";
    runAI(points, "示范数据");
  }

  async function usePublicData() {
    let text = PUBLIC_CSV;
    try {
      const response = await fetch("../data/public_tt3d_sample.csv");
      if (response.ok) text = await response.text();
    } catch (error) {
      text = PUBLIC_CSV;
    }
    const points = parseCSV(text);
    state.observed = points;
    $("uploadName").textContent = "public_tt3d_sample.csv · TT3D公开轨迹";
    runAI(points, "TT3D公开轨迹");
  }

  function startAnimation() {
    render();
    if (!state.current) render();
    if (state.animationId) cancelAnimationFrame(state.animationId);
    const points = state.current.full.points;
    const duration = points[points.length - 1].t;
    const startedAt = performance.now();
    const tick = (now) => {
      const progress = clamp((now - startedAt) / 1800, 0, 1);
      drawTrajectory(interpolate(points, progress * duration));
      if (progress < 1) state.animationId = requestAnimationFrame(tick); else { state.animationId = null; updateStatus("飞行完成"); }
    };
    updateStatus("正在播放", false);
    state.animationId = requestAnimationFrame(tick);
  }

  function bindControls() {
    $("ballType").addEventListener("change", (event) => { state.ball = event.target.value; render(); });
    $("speedRange").addEventListener("input", (event) => { state.speed = Number(event.target.value); updateLabels(); render(); });
    $("angleRange").addEventListener("input", (event) => { state.angle = Number(event.target.value); updateLabels(); render(); });
    $("spinRange").addEventListener("input", (event) => { state.spin = Number(event.target.value); updateLabels(); render(); });
    $("windRange").addEventListener("input", (event) => { state.wind = Number(event.target.value); updateLabels(); render(); });
    $("physicsMode").addEventListener("change", (event) => { state.physicsMode = event.target.value; render(); });
    $("densityRange").addEventListener("input", (event) => { state.density = Number(event.target.value); updateLabels(); render(); });
    $("spinDecayRange").addEventListener("input", (event) => { state.spinDecay = Number(event.target.value); updateLabels(); render(); });
    $("gustRange").addEventListener("input", (event) => { state.gust = Number(event.target.value); updateLabels(); render(); });
    $("runButton").addEventListener("click", () => { updateStatus("正在计算", false); window.requestAnimationFrame(() => render()); });
    $("resetButton").addEventListener("click", () => { state.ball = "tennis"; state.speed = 22; state.angle = 18; state.spin = 1200; state.wind = 0; state.density = 1.225; state.spinDecay = 0.4; state.gust = 0; state.physicsMode = "advanced"; $("ballType").value = state.ball; $("speedRange").value = state.speed; $("angleRange").value = state.angle; $("spinRange").value = state.spin; $("windRange").value = state.wind; $("densityRange").value = state.density; $("spinDecayRange").value = state.spinDecay; $("gustRange").value = state.gust; $("physicsMode").value = state.physicsMode; document.querySelectorAll(".case-button").forEach((button) => button.classList.toggle("active", button.dataset.case === "baseline")); render(); });
    $("animateButton").addEventListener("click", startAnimation);
    $("heroStart").addEventListener("click", () => $("simulator").scrollIntoView({ behavior: "smooth" }));
    $("aiRunButton").addEventListener("click", () => runAI(state.observed || state.current.full.points));
    $("usePublicDataButton").addEventListener("click", usePublicData);
    $("useDemoButton").addEventListener("click", useDemoData);
    $("csvInput").addEventListener("change", (event) => {
      const file = event.target.files && event.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        const points = parseCSV(String(reader.result));
        state.observed = points;
        $("uploadName").textContent = `${file.name} · ${points.length} 个点`;
        runAI(points, file.name);
      };
      reader.readAsText(file);
    });
    document.querySelectorAll(".nav-link").forEach((button) => button.addEventListener("click", () => $(button.dataset.target).scrollIntoView({ behavior: "smooth" })));
    document.querySelectorAll(".case-button").forEach((button) => button.addEventListener("click", () => {
      const presets = {
        baseline: { wind: 0, gust: 0, density: 1.225, spinDecay: 0.4 },
        crosswind: { wind: 4, gust: 1.8, density: 1.225, spinDecay: 0.4 },
        spinloss: { wind: 0, gust: 0.7, density: 1.225, spinDecay: 1.35 },
        density: { wind: 0, gust: 0, density: 0.96, spinDecay: 0.4 }
      };
      const preset = presets[button.dataset.case];
      if (!preset) return;
      state.wind = preset.wind; state.gust = preset.gust; state.density = preset.density; state.spinDecay = preset.spinDecay;
      $("windRange").value = state.wind; $("gustRange").value = state.gust; $("densityRange").value = state.density; $("spinDecayRange").value = state.spinDecay;
      document.querySelectorAll(".case-button").forEach((item) => item.classList.toggle("active", item === button));
      render();
    }));
    window.addEventListener("resize", () => { if (state.current) { drawTrajectory(); if (state.observed) runAI(state.observed, "当前轨迹"); } });
    setupNavSpy();
  }

  bindControls();
  render();
})();

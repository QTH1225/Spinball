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
    angle: 45,
    spin: 1200,
    wind: 0,
    density: 1.225,
    spinDecay: 0.4,
    gust: 0,
    physicsMode: "advanced",
    spaceMode: "3d",
    axisTilt: 35,
    axisAzimuth: 90,
    lateralWind: 0,
    dt: 0.008,
    current: null,
    observed: null,
    observedSource: "current",
    observedRequestId: 0,
    aiResult: null,
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
      axisTilt: config.axisTilt ?? state.axisTilt,
      axisAzimuth: config.axisAzimuth ?? state.axisAzimuth,
      lateralWind: config.lateralWind ?? state.lateralWind,
      diameter: ball.radius * 2,
      spinRad: (config.spin ?? state.spin) * (2 * Math.PI / 60)
    };
  }

  function windAt(y, t, params) {
    const gust = params.physicsMode === "advanced" ? params.gust * Math.sin(2 * Math.PI * 0.65 * t + 0.12 * Math.max(y, 0)) : 0;
    const shear = params.physicsMode === "advanced" ? 0.025 * params.wind * Math.max(y, 0) : 0;
    return params.wind + gust + shear;
  }

  function windVectorAt(y, t, params) {
    const lateralGust = params.physicsMode === "advanced" ? 0.35 * params.gust * Math.cos(2 * Math.PI * 0.55 * t) : 0;
    return { x: windAt(y, t, params), y: 0, z: params.lateralWind + lateralGust };
  }

  function spinVectorAt(params, t) {
    const spinRad = params.physicsMode === "advanced" ? params.spinRad * Math.exp(-params.spinDecay * t) : params.spinRad;
    const tilt = params.axisTilt * DEG;
    const azimuth = params.axisAzimuth * DEG;
    return {
      x: spinRad * Math.sin(tilt) * Math.cos(azimuth),
      y: spinRad * Math.sin(tilt) * Math.sin(azimuth),
      z: spinRad * Math.cos(tilt)
    };
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

  function aerodynamicState3D(s, params, t) {
    const density = params.physicsMode === "advanced"
      ? params.density * Math.exp(-G * Math.max(s.y, 0) / (GAS_CONSTANT * AIR_TEMPERATURE))
      : params.density;
    const wind = windVectorAt(s.y, t, params);
    const relativeVx = s.vx - wind.x;
    const relativeVy = s.vy - wind.y;
    const relativeVz = s.vz - wind.z;
    const relativeSpeed = Math.hypot(relativeVx, relativeVy, relativeVz);
    const spinVector = spinVectorAt(params, t);
    const spinRad = Math.hypot(spinVector.x, spinVector.y, spinVector.z);
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
    return { density, relativeVx, relativeVy, relativeVz, relativeSpeed, spinRad, spinParameter, reynolds, cd, cl, spinVector, buoyancyForce, dragForce: dynamicPressure * params.area * cd, magnusForce: dynamicPressure * params.area * Math.abs(cl) };
  }

  function derivative3D(s, params, mode, t) {
    let ax = 0;
    let ay = -G;
    let az = 0;

    if (mode !== "vacuum") {
      const aero = aerodynamicState3D(s, params, t);
      if (aero.relativeSpeed > 1e-8) {
        const dragFactor = 0.5 * aero.density * aero.cd * params.area * aero.relativeSpeed / params.mass;
        ax -= dragFactor * aero.relativeVx;
        ay -= dragFactor * aero.relativeVy;
        az -= dragFactor * aero.relativeVz;
        ay += aero.buoyancyForce / params.mass;

        if (mode === "full" && Math.abs(aero.spinParameter) > 1e-8) {
          const crossX = aero.spinVector.y * aero.relativeVz - aero.spinVector.z * aero.relativeVy;
          const crossY = aero.spinVector.z * aero.relativeVx - aero.spinVector.x * aero.relativeVz;
          const crossZ = aero.spinVector.x * aero.relativeVy - aero.spinVector.y * aero.relativeVx;
          const crossMagnitude = Math.hypot(crossX, crossY, crossZ);
          if (crossMagnitude > 1e-8) {
            const magnusFactor = 0.5 * aero.density * params.area * aero.cl * aero.relativeSpeed * aero.relativeSpeed / params.mass;
            ax += magnusFactor * crossX / crossMagnitude;
            ay += magnusFactor * crossY / crossMagnitude;
            az += magnusFactor * crossZ / crossMagnitude;
          }
        }
      }
    }

    return { x: s.vx, y: s.vy, z: s.vz, vx: ax, vy: ay, vz: az };
  }

  function addState3D(a, b, scale) {
    return { x: a.x + b.x * scale, y: a.y + b.y * scale, z: a.z + b.z * scale, vx: a.vx + b.vx * scale, vy: a.vy + b.vy * scale, vz: a.vz + b.vz * scale };
  }

  function rk4Step3D(s, params, dt, mode, t) {
    const k1 = derivative3D(s, params, mode, t);
    const k2 = derivative3D(addState3D(s, k1, dt / 2), params, mode, t + dt / 2);
    const k3 = derivative3D(addState3D(s, k2, dt / 2), params, mode, t + dt / 2);
    const k4 = derivative3D(addState3D(s, k3, dt), params, mode, t + dt);
    return {
      x: s.x + (dt / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
      y: s.y + (dt / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
      z: s.z + (dt / 6) * (k1.z + 2 * k2.z + 2 * k3.z + k4.z),
      vx: s.vx + (dt / 6) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
      vy: s.vy + (dt / 6) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy),
      vz: s.vz + (dt / 6) * (k1.vz + 2 * k2.vz + 2 * k3.vz + k4.vz)
    };
  }

  function simulate3D(config = {}, mode = "full") {
    const speed = config.speed ?? state.speed;
    const angle = (config.angle ?? state.angle) * DEG;
    const integrationStep = config.dt ?? state.dt;
    const params = ballParams(config);
    let s = { x: 0, y: 0, z: 0, vx: speed * Math.cos(angle), vy: speed * Math.sin(angle), vz: 0 };
    const points = [{ t: 0, x: s.x, y: s.y, z: s.z, vx: s.vx, vy: s.vy, vz: s.vz }];
    const initialAero = aerodynamicState3D(s, params, 0);
    let t = 0;
    const maxTime = 8;

    for (let step = 0; step < maxTime / integrationStep; step += 1) {
      const previous = s;
      s = rk4Step3D(s, params, integrationStep, mode, t);
      t += integrationStep;
      if (s.y < 0 && t > integrationStep) {
        const fraction = previous.y / Math.max(previous.y - s.y, 1e-9);
        const landing = {
          t: t - integrationStep + integrationStep * fraction,
          x: lerp(previous.x, s.x, fraction),
          y: 0,
          z: lerp(previous.z, s.z, fraction),
          vx: lerp(previous.vx, s.vx, fraction),
          vy: lerp(previous.vy, s.vy, fraction),
          vz: lerp(previous.vz, s.vz, fraction)
        };
        points.push(landing);
        break;
      }
      points.push({ t, x: s.x, y: Math.max(0, s.y), z: s.z, vx: s.vx, vy: s.vy, vz: s.vz });
    }

    const maxHeight = Math.max(...points.map((point) => point.y));
    const last = points[points.length - 1];
    const finalAero = aerodynamicState3D(last, params, last.t);
    return { points, range: Math.hypot(last.x, last.z), sideRange: last.z, height: maxHeight, time: last.t, params, mode, dt: integrationStep, diagnostics: { initial: initialAero, final: finalAero, spinEnd: finalAero.spinRad * 60 / (2 * Math.PI) } };
  }

  function simulate(config = {}, mode = "full") {
    const speed = config.speed ?? state.speed;
    const angle = (config.angle ?? state.angle) * DEG;
    const integrationStep = config.dt ?? state.dt;
    const params = ballParams(config);
    let s = { x: 0, y: 0, vx: speed * Math.cos(angle), vy: speed * Math.sin(angle) };
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
    return { points, range: last.x, sideRange: 0, height: maxHeight, time: last.t, params, mode, dt: integrationStep, diagnostics: { initial: initialAero, final: finalAero, spinEnd: finalAero.spinRad * 60 / (2 * Math.PI) } };
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

  function fitSamplePoints(points, maximumPoints = 80) {
    if (points.length <= maximumPoints) return points;
    const sampled = [];
    const stride = (points.length - 1) / (maximumPoints - 1);
    for (let index = 0; index < maximumPoints; index += 1) {
      sampled.push(points[Math.round(index * stride)]);
    }
    return sampled;
  }

  function fitObjective(points, candidate, integrationStep) {
    const replay = simulate({
      ball: state.ball,
      speed: candidate.speed,
      angle: candidate.angle,
      spin: candidate.spin,
      cd: candidate.cd,
      wind: state.wind,
      density: state.density,
      spinDecay: state.spinDecay,
      gust: state.gust,
      physicsMode: state.physicsMode,
      dt: integrationStep
    }, "full");
    const observedEnd = points[points.length - 1].t;
    const replayEnd = replay.points[replay.points.length - 1].t;
    let squaredError = 0;
    points.forEach((point, index) => {
      const prediction = interpolate(replay.points, point.t);
      const positionError = Math.hypot(point.x - prediction.x, point.y - prediction.y);
      const weight = index === points.length - 1 ? 1.35 : 1;
      squaredError += weight * positionError * positionError;
    });
    const durationPenalty = replayEnd + integrationStep < observedEnd
      ? 0.65 * (observedEnd - replayEnd)
      : 0;
    const rmse = Math.sqrt(squaredError / Math.max(points.length, 1));
    return { score: rmse + durationPenalty, rmse, replay };
  }

  function clampFitCandidate(candidate, bounds) {
    return {
      speed: clamp(candidate.speed, bounds.speed.min, bounds.speed.max),
      angle: clamp(candidate.angle, bounds.angle.min, bounds.angle.max),
      spin: clamp(candidate.spin, bounds.spin.min, bounds.spin.max),
      cd: clamp(candidate.cd, bounds.cd.min, bounds.cd.max)
    };
  }

  function optimizeFitStart(start, points, integrationStep, bounds) {
    const parameters = ["speed", "angle", "spin", "cd"];
    const initialSteps = { speed: 4, angle: 8, spin: 800, cd: 0.16 };
    const makeCandidate = (values) => clampFitCandidate({
      speed: values[0], angle: values[1], spin: values[2], cd: values[3]
    }, bounds);
    const toVector = (candidate) => parameters.map((parameter) => candidate[parameter]);
    const evaluate = (vector) => {
      const candidate = makeCandidate(vector);
      const evaluation = fitObjective(points, candidate, integrationStep);
      return { vector: toVector(candidate), candidate, ...evaluation };
    };
    const initial = clampFitCandidate(start, bounds);
    const simplex = [evaluate(toVector(initial))];
    parameters.forEach((parameter, index) => {
      const vector = toVector(initial);
      vector[index] += initialSteps[parameter];
      simplex.push(evaluate(vector));
    });
    const compare = (first, second) => first.score - second.score;
    const vectorOperation = (first, second, scale = 1) => first.map((value, index) => value + (second[index] - value) * scale);
    const maxIterations = 72;
    for (let iteration = 0; iteration < maxIterations; iteration += 1) {
      simplex.sort(compare);
      const best = simplex[0];
      const worst = simplex[simplex.length - 1];
      const secondWorst = simplex[simplex.length - 2];
      const centroid = parameters.map((_, index) => simplex.slice(0, -1).reduce((sum, item) => sum + item.vector[index], 0) / parameters.length);
      const reflected = evaluate(vectorOperation(centroid, worst.vector, -1));
      if (reflected.score < best.score) {
        const expanded = evaluate(vectorOperation(centroid, reflected.vector, 2));
        simplex[simplex.length - 1] = expanded.score < reflected.score ? expanded : reflected;
      } else if (reflected.score < secondWorst.score) {
        simplex[simplex.length - 1] = reflected;
      } else {
        const contracted = evaluate(vectorOperation(centroid, worst.vector, 0.5));
        if (contracted.score < worst.score) {
          simplex[simplex.length - 1] = contracted;
        } else {
          for (let index = 1; index < simplex.length; index += 1) {
            simplex[index] = evaluate(best.vector.map((value, vectorIndex) => value + (simplex[index].vector[vectorIndex] - value) * 0.5));
          }
        }
      }
      const spread = Math.max(...simplex.map((item) => Math.abs(item.score - best.score)));
      if (spread < 1e-5) break;
    }
    simplex.sort(compare);
    const result = simplex[0];
    return { ...result.candidate, score: result.score, rmse: result.rmse, replay: result.replay };
  }

  function fitTrajectory(points) {
    if (!points || points.length < 8) return null;
    const sampled = fitSamplePoints(points);
    const timeGaps = sampled.slice(1)
      .map((point, index) => point.t - sampled[index].t)
      .filter((gap) => gap > 1e-5)
      .sort((first, second) => first - second);
    const medianGap = timeGaps[Math.floor(timeGaps.length / 2)] || state.dt;
    const integrationStep = clamp(Math.min(state.dt, medianGap / 2), 0.002, 0.01);
    const bounds = {
      speed: { min: 8, max: 36 },
      angle: { min: 5, max: 75 },
      spin: { min: -2600, max: 2600 },
      cd: { min: 0.12, max: 0.8 }
    };
    const firstGap = sampled[Math.min(2, sampled.length - 1)];
    const startDt = Math.max(firstGap.t - sampled[0].t, 1e-5);
    const startVx = (firstGap.x - sampled[0].x) / startDt;
    const startVy = (firstGap.y - sampled[0].y) / startDt;
    const speedGuess = clamp(Math.hypot(startVx, startVy) * 1.12, bounds.speed.min, bounds.speed.max);
    const angleGuess = clamp(Math.atan2(startVy, Math.max(startVx, 1e-5)) / DEG, bounds.angle.min, bounds.angle.max);
    const baseCd = BALLS[state.ball].cd;
    const gridStarts = [];
    [10, 18, 26, 34].forEach((speed) => [10, 30, 50, 70].forEach((angle) => [-2600, -1300, 0, 1300, 2600].forEach((spin) => [0.18, 0.38, 0.58, 0.78].forEach((cd) => {
      const candidate = { speed, angle, spin, cd };
      gridStarts.push({ ...candidate, ...fitObjective(sampled, candidate, integrationStep) });
    }))));
    gridStarts.push(...[
      { speed: speedGuess, angle: angleGuess, spin: state.spin, cd: baseCd },
      { speed: 20, angle: 40, spin: 1200, cd: baseCd },
      { speed: 28, angle: 52, spin: -1200, cd: 0.42 }
    ].map((candidate) => ({ ...candidate, ...fitObjective(sampled, candidate, integrationStep) })));
    const starts = gridStarts
      .sort((first, second) => first.score - second.score)
      .slice(0, 5);
    const candidates = starts.map((start) => optimizeFitStart(start, sampled, integrationStep, bounds));
    const best = candidates.sort((first, second) => first.score - second.score)[0];
    const confidenceScale = Math.max(0.02, Math.max(...sampled.map((point) => Math.hypot(point.x, point.y))) * 0.025);
    return {
      speed: best.speed,
      angle: best.angle,
      spin: best.spin,
      cd: best.cd,
      confidence: clamp(Math.exp(-best.rmse / confidenceScale), 0, 0.99),
      neighborDistance: best.rmse,
      rmse: best.rmse,
      method: "direct-fit",
      replay: best.replay
    };
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
        const point = { t: time, x: lerp(a.x, b.x, ratio), y: lerp(a.y, b.y, ratio) };
        if (a.z !== undefined || b.z !== undefined) point.z = lerp(a.z ?? 0, b.z ?? 0, ratio);
        return point;
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

  function replayDuringObservation(observed, replay) {
    const lastTime = observed[observed.length - 1].t;
    const visible = replay.filter((point) => point.t <= lastTime);
    if (visible.length && visible[visible.length - 1].t < lastTime) {
      visible.push(interpolate(replay, lastTime));
    }
    return visible;
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

  function drawInfoCallout(targetContext, width, height, x, y, lines, color, align = "left") {
    targetContext.save();
    targetContext.font = "600 11px Manrope, 'Microsoft YaHei', sans-serif";
    const paddingX = 8;
    const lineHeight = 16;
    const boxHeight = lines.length * lineHeight + 10;
    const boxWidth = Math.max(...lines.map((line) => targetContext.measureText(line).width)) + paddingX * 2;
    const boxX = clamp(align === "right" ? x - boxWidth : x, 4, width - boxWidth - 4);
    const boxY = clamp(y - boxHeight, 4, height - boxHeight - 4);
    targetContext.fillStyle = "rgba(255,255,255,.94)";
    targetContext.strokeStyle = color;
    targetContext.lineWidth = 1;
    targetContext.fillRect(boxX, boxY, boxWidth, boxHeight);
    targetContext.strokeRect(boxX, boxY, boxWidth, boxHeight);
    targetContext.fillStyle = color;
    targetContext.textAlign = "left";
    lines.forEach((line, index) => targetContext.fillText(line, boxX + paddingX, boxY + 15 + index * lineHeight));
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

  function drawGrid3D(targetContext, width, height, bounds) {
    const pad = { left: 62, right: 26, top: 28, bottom: 54 };
    const availableWidth = width - pad.left - pad.right;
    const availableHeight = height - pad.top - pad.bottom;
    const zMin = bounds.zMin;
    const zMax = bounds.zMax;
    const sideVisualScale = 2.35;
    const yAxisX = -Math.SQRT1_2;
    const yAxisY = Math.SQRT1_2;
    const projectedX = (x, z) => x + yAxisX * z * sideVisualScale;
    const projectedY = (z, vertical) => yAxisY * z * sideVisualScale - vertical;
    const projectedXMin = Math.min(projectedX(0, zMin), projectedX(0, zMax));
    const projectedXMax = Math.max(projectedX(bounds.xMax, zMin), projectedX(bounds.xMax, zMax));
    const projectedYMin = Math.min(projectedY(zMin, bounds.yMax), projectedY(zMax, bounds.yMax));
    const projectedYMax = Math.max(projectedY(zMin, 0), projectedY(zMax, 0));
    const scale = Math.min(
      availableWidth / Math.max(projectedXMax - projectedXMin, 1e-6),
      availableHeight / Math.max(projectedYMax - projectedYMin, 1e-6)
    );
    const map3D = (point) => {
      const z = point.z ?? 0;
      return {
        x: pad.left + (projectedX(point.x, z) - projectedXMin) * scale,
        y: pad.top + (projectedY(z, point.y) - projectedYMin) * scale
      };
    };
    const drawLine = (points, strokeStyle, lineWidth = 1, dash = []) => {
      targetContext.beginPath();
      targetContext.strokeStyle = strokeStyle;
      targetContext.lineWidth = lineWidth;
      targetContext.setLineDash(dash);
      points.forEach((point, index) => {
        if (index === 0) targetContext.moveTo(point.x, point.y); else targetContext.lineTo(point.x, point.y);
      });
      targetContext.stroke();
      targetContext.setLineDash([]);
    };
    const groundPoint = (x, z) => map3D({ x, y: 0, z });
    const origin = groundPoint(0, 0);
    const xAxisEnd = groundPoint(bounds.xMax, 0);
    const yAxisEnd = groundPoint(0, zMax);
    const zAxisEnd = map3D({ x: 0, y: bounds.yMax, z: 0 });
    const drawArrow = (start, end, strokeStyle, lineWidth) => {
      drawLine([start, end], strokeStyle, lineWidth);
      const angle = Math.atan2(end.y - start.y, end.x - start.x);
      const arrowLength = 8;
      const left = { x: end.x - arrowLength * Math.cos(angle - Math.PI / 6), y: end.y - arrowLength * Math.sin(angle - Math.PI / 6) };
      const right = { x: end.x - arrowLength * Math.cos(angle + Math.PI / 6), y: end.y - arrowLength * Math.sin(angle + Math.PI / 6) };
      drawLine([left, end, right], strokeStyle, lineWidth);
    };
    targetContext.clearRect(0, 0, width, height);
    targetContext.fillStyle = "#f0f5ef";
    targetContext.fillRect(0, 0, width, height);
    targetContext.fillStyle = "rgba(185,247,196,.18)";
    targetContext.beginPath();
    [groundPoint(0, zMin), groundPoint(bounds.xMax, zMin), groundPoint(bounds.xMax, zMax), groundPoint(0, zMax)].forEach((point, index) => {
      if (index === 0) targetContext.moveTo(point.x, point.y); else targetContext.lineTo(point.x, point.y);
    });
    targetContext.closePath();
    targetContext.fill();
    for (let index = 0; index <= 6; index += 1) {
      const x = (index / 6) * bounds.xMax;
      drawLine([groundPoint(x, zMin), groundPoint(x, zMax)], "rgba(35,76,61,.12)", 1, [3, 5]);
    }
    for (let index = 0; index <= 5; index += 1) {
      const z = zMin + (index / 5) * (zMax - zMin);
      drawLine([groundPoint(0, z), groundPoint(bounds.xMax, z)], "rgba(35,76,61,.12)", 1, [3, 5]);
    }
    drawLine([groundPoint(0, zMin), groundPoint(0, zMax)], "rgba(23,63,53,.28)", 1);
    drawArrow(origin, xAxisEnd, "rgba(23,63,53,.62)", 1.6);
    drawArrow(origin, yAxisEnd, "rgba(23,63,53,.62)", 1.6);
    drawArrow(origin, zAxisEnd, "rgba(23,63,53,.62)", 1.6);
    targetContext.fillStyle = "#82948b";
    targetContext.font = "11px DM Mono, monospace";
    targetContext.textAlign = "center";
    for (let index = 0; index <= 6; index += 1) {
      const ratio = index / 6;
      const point = groundPoint(ratio * bounds.xMax, 0);
      targetContext.fillText(formatAxisTick(ratio * bounds.xMax), point.x, point.y + 19);
    }
    for (let index = 0; index <= 4; index += 1) {
      const ratio = index / 4;
      const z = zMin + ratio * (zMax - zMin);
      if (Math.abs(z) < 1e-8) continue;
      const point = groundPoint(0, z);
      targetContext.fillText(formatAxisTick(z), point.x - 8, point.y + 4);
    }
    targetContext.textAlign = "right";
    for (let index = 0; index <= 4; index += 1) {
      const ratio = index / 4;
      if (index === 0) continue;
      const point = map3D({ x: 0, y: ratio * bounds.yMax, z: 0 });
      targetContext.fillText(formatAxisTick(ratio * bounds.yMax), point.x - 9, point.y + 4);
    }
    targetContext.font = "600 11px Manrope, 'Microsoft YaHei', sans-serif";
    targetContext.fillStyle = "#48675b";
    targetContext.textAlign = "right";
    const xLabel = xAxisEnd;
    targetContext.fillText("X / m", xLabel.x - 2, xLabel.y + 38);
    targetContext.textAlign = "left";
    targetContext.fillText("Y / m", yAxisEnd.x - 26, yAxisEnd.y + 18);
    targetContext.fillText("Z / m", zAxisEnd.x + 8, zAxisEnd.y - 8);
    return { map3D, groundPoint, pad };
  }

  function drawPath3D(targetContext, points, map3D, color, width, dashed = false) {
    if (!points || points.length < 2) return;
    targetContext.beginPath();
    targetContext.lineWidth = width;
    targetContext.strokeStyle = color;
    targetContext.lineJoin = "round";
    targetContext.lineCap = "round";
    targetContext.setLineDash(dashed ? [5, 6] : []);
    points.forEach((point, index) => {
      const mapped = map3D(point);
      if (index === 0) targetContext.moveTo(mapped.x, mapped.y); else targetContext.lineTo(mapped.x, mapped.y);
    });
    targetContext.stroke();
    targetContext.setLineDash([]);
  }

  function drawTrajectory3D(marker = null) {
    const size = canvasSize(canvas, ctx);
    const all = [...state.current.vacuum.points, ...state.current.drag.points, ...state.current.full.points];
    const xMax = Math.max(10, ...all.map((point) => point.x)) * 1.06;
    const yMax = Math.max(3, ...all.map((point) => point.y)) * 1.12;
    const zExtent = Math.max(0.8, ...all.map((point) => Math.abs(point.z ?? 0))) * 1.22;
    const mapper = drawGrid3D(ctx, size.width, size.height, { xMax, yMax, zMin: -zExtent, zMax: zExtent });
    drawPath3D(ctx, state.current.vacuum.points, mapper.map3D, "#b6c7bc", 1.8, true);
    drawPath3D(ctx, state.current.drag.points, mapper.map3D, "#f17e55", 2.1);
    drawPath3D(ctx, state.current.full.points, mapper.map3D, "#4a87ff", 2.8);
    const fullPoints = state.current.full.points;
    const landing = fullPoints[fullPoints.length - 1];
    const peak = fullPoints.reduce((highest, point) => point.y > highest.y ? point : highest, fullPoints[0]);
    const landingPoint = mapper.map3D(landing);
    const peakPoint = mapper.map3D(peak);
    const landingGround = mapper.groundPoint(landing.x, landing.z ?? 0);
    const peakGround = mapper.groundPoint(peak.x, peak.z ?? 0);
    const originGround = mapper.groundPoint(0, 0);
    const landingXProjection = mapper.groundPoint(landing.x, 0);
    const landingYProjection = mapper.groundPoint(0, landing.z ?? 0);
    const peakXProjection = mapper.groundPoint(peak.x, 0);
    const peakYProjection = mapper.groundPoint(0, peak.z ?? 0);
    const landingXYDistance = Math.hypot(landing.x, landing.z ?? 0);
    const peakXYDistance = Math.hypot(peak.x, peak.z ?? 0);
    const signedProjection = (value) => `${value >= 0 ? "+" : ""}${value.toFixed(2)}`;
    ctx.save();
    ctx.setLineDash([3, 4]);
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(241,126,85,.58)";
    ctx.beginPath(); ctx.moveTo(landingPoint.x, landingPoint.y); ctx.lineTo(landingGround.x, landingGround.y); ctx.stroke();
    ctx.strokeStyle = "rgba(74,135,255,.5)";
    ctx.beginPath(); ctx.moveTo(peakPoint.x, peakPoint.y); ctx.lineTo(peakGround.x, peakGround.y); ctx.stroke();
    ctx.strokeStyle = "rgba(217,95,61,.48)";
    ctx.beginPath(); ctx.moveTo(originGround.x, originGround.y); ctx.lineTo(landingGround.x, landingGround.y); ctx.stroke();
    ctx.strokeStyle = "rgba(117,94,232,.48)";
    ctx.beginPath(); ctx.moveTo(originGround.x, originGround.y); ctx.lineTo(peakGround.x, peakGround.y); ctx.stroke();
    ctx.setLineDash([2, 3]);
    ctx.strokeStyle = "rgba(217,95,61,.7)";
    ctx.beginPath(); ctx.moveTo(landingGround.x, landingGround.y); ctx.lineTo(landingXProjection.x, landingXProjection.y); ctx.stroke();
    ctx.strokeStyle = "rgba(117,94,232,.7)";
    ctx.beginPath(); ctx.moveTo(landingGround.x, landingGround.y); ctx.lineTo(landingYProjection.x, landingYProjection.y); ctx.stroke();
    ctx.strokeStyle = "rgba(217,95,61,.7)";
    ctx.beginPath(); ctx.moveTo(peakGround.x, peakGround.y); ctx.lineTo(peakXProjection.x, peakXProjection.y); ctx.stroke();
    ctx.strokeStyle = "rgba(117,94,232,.7)";
    ctx.beginPath(); ctx.moveTo(peakGround.x, peakGround.y); ctx.lineTo(peakYProjection.x, peakYProjection.y); ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "#f17e55";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(landingPoint.x, landingPoint.y, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#4a87ff";
    ctx.beginPath(); ctx.arc(peakPoint.x, peakPoint.y, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#755ee8";
    ctx.beginPath(); ctx.arc(peakGround.x, peakGround.y, 4, 0, Math.PI * 2); ctx.fill();
    const landingCalloutY = Math.min(landingPoint.y + 62, size.height - 6);
    drawInfoCallout(ctx, size.width, size.height, landingPoint.x + 18, landingCalloutY, [
      `落点距原点 ${landingXYDistance.toFixed(2)} m`,
      `X ${landing.x.toFixed(2)} m · Y ${signedProjection(landing.z ?? 0)} m`
    ], "#d95f3d");
    drawInfoCallout(ctx, size.width, size.height, peakPoint.x + 10, peakPoint.y - 6, [
      `最高点 Z ${peak.y.toFixed(2)} m`,
      `XY投影距原点 ${peakXYDistance.toFixed(2)} m`,
      `X投影 ${peak.x.toFixed(2)} m · Y投影 ${signedProjection(peak.z ?? 0)} m`
    ], "#356dcc");
    const start = mapper.map3D({ x: 0, y: 0, z: 0 });
    ctx.fillStyle = "#173f35";
    ctx.beginPath(); ctx.arc(start.x, start.y, 4, 0, Math.PI * 2); ctx.fill();
    if (marker) {
      const markerPoint = mapper.map3D(marker);
      ctx.fillStyle = "#d9ff68"; ctx.strokeStyle = "#173f35"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(markerPoint.x, markerPoint.y, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
  }

  function drawTrajectory(marker = null) {
    if (!state.current) return;
    if (state.spaceMode === "3d") {
      drawTrajectory3D(marker);
      return;
    }
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
    const startX = mapper.mapX(0); const startY = mapper.mapY(0);
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
    const yMin = 0;
    const yMax = Math.max(1.5, ...all.map((point) => point.y)) * 1.18;
    const mapper = drawGrid(replayCtx, size.width, size.height, { xMin: 0, xMax, yMin, yMax });
    drawPath(replayCtx, replay, mapper.mapX, mapper.mapY, "#4a87ff", 2.4);
    replayCtx.fillStyle = "#f17e55";
    observed.forEach((point) => { replayCtx.beginPath(); replayCtx.arc(mapper.mapX(point.x), mapper.mapY(point.y), 3.2, 0, Math.PI * 2); replayCtx.fill(); });
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
    $("axisTiltValue").textContent = `${state.axisTilt.toFixed(0)}°`;
    $("axisAzimuthValue").textContent = `${state.axisAzimuth.toFixed(0)}°`;
    $("lateralWindValue").textContent = `${state.lateralWind >= 0 ? "+" : ""}${state.lateralWind.toFixed(1)} m/s`;
    $("coordinateTag").textContent = state.spaceMode === "3d" ? "x / y / z · m" : "x / y · m";
    $("stageTitle").textContent = state.spaceMode === "3d"
      ? "三维空间：旋转轴与侧风共同改变轨迹"
      : state.spin > 100 ? "正旋转：升力抬高轨迹" : state.spin < -100 ? "反旋转：轨迹更快下沉" : "无明显旋转：接近抛体运动";
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
    $("sideValue").textContent = Math.abs(full.sideRange).toFixed(2);
    const contribution = full.height - drag.height;
    $("landingValue").textContent = `${contribution >= 0 ? "+" : ""}${contribution.toFixed(2)}`;
    const diagnostic = full.diagnostics.initial;
    $("reynoldsValue").textContent = diagnostic.reynolds >= 1000 ? `${(diagnostic.reynolds / 1000).toFixed(1)}k` : diagnostic.reynolds.toFixed(0);
    $("spinParameterValue").textContent = diagnostic.spinParameter.toFixed(3);
    $("liftCoefficientValue").textContent = diagnostic.cl.toFixed(3);
    $("dynamicCdValue").textContent = diagnostic.cd.toFixed(3);
    $("forceRatioValue").textContent = diagnostic.dragForce > 1e-8 ? (diagnostic.magnusForce / diagnostic.dragForce).toFixed(2) : "—";
    $("buoyancyValue").textContent = `${(diagnostic.buoyancyForce / (BALLS[state.ball].mass * G) * 100).toFixed(2)}%`;
    $("modelLimits").textContent = state.spaceMode === "3d"
      ? state.physicsMode === "advanced" ? "三维空间 · 旋转轴可倾斜 · 侧向风与动态气动" : "三维空间 · 常系数 Cd/Cl · 侧向风"
      : state.physicsMode === "advanced" ? "二维平面 · 风场/旋转时变 · 光滑球 Cd(Re) 或球类代表 Cd" : "二维平面 · 常系数 Cd/Cl · 无三维旋转轴";
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
      const exact = state.speed * Math.sin(angle) * point.t - 0.5 * G * point.t * point.t;
      maxError = Math.max(maxError, Math.abs(point.y - Math.max(0, exact)));
    });
    return (maxError / Math.max(maxY, 1)) * 100;
  }

  function integrationRefinementError(coarse, fine) {
    const errors = coarse.points.map((point) => {
      const refined = interpolate(fine.points, point.t);
      return Math.hypot(point.x - refined.x, point.y - refined.y, (point.z ?? 0) - (refined.z ?? 0));
    });
    return Math.sqrt(errors.reduce((sum, error) => sum + error * error, 0) / errors.length);
  }

  function render() {
    updateLabels();
    const simulateCurrent = state.spaceMode === "3d" ? simulate3D : simulate;
    state.current = {
      vacuum: simulateCurrent({}, "vacuum"),
      drag: simulateCurrent({}, "drag"),
      full: simulateCurrent({}, "full")
    };
    state.current.refined = simulateCurrent({ dt: state.dt / 2 }, "full");
    if (!state.observed || state.observedSource === 'current') {
      state.observed = state.current.full.points;
      state.observedSource = 'current';
      setAIInputSource('current');
    }
    updateMetrics();
    drawTrajectory();
    $("canvasEmpty").classList.add("hidden");
    $("analyticError").textContent = analyticError() < 0.1 ? "通过" : "需检查";
    $("dtError").textContent = integrationRefinementError(state.current.full, state.current.refined) < 0.001 ? "稳定" : "需检查";
    $("energyNote").textContent = state.physicsMode === "advanced" ? "已检查" : "基础模式";
    updateCaseReadout();
    updateStatus("模型已更新");
  }

  function setAIInputSource(source) {
    document.querySelectorAll(".ai-source-button").forEach((button) => {
      const active = button.dataset.source === source;
      button.classList.toggle("active", active);
      button.setAttribute("aria-pressed", String(active));
    });
  }

  function queueAI(points, source, requestId = state.observedRequestId) {
    updateStatus("正在反演", false);
    window.setTimeout(() => {
      if (requestId === state.observedRequestId) runAI(points, source);
    }, 0);
  }
  function runAI(points = state.observed, source = "当前轨迹") {
    const directFit = state.observedSource === "upload" || state.observedSource === "demo";
    const estimate = directFit ? fitTrajectory(points) : infer(points);
    if (!estimate) {
      $("aiMessage").textContent = "轨迹点不足或格式无效，至少需要 8 个有效坐标。";
      return;
    }
    const replay = estimate.replay || (state.spaceMode === "3d" ? simulate3D : simulate)({ ball: state.ball, speed: estimate.speed, angle: estimate.angle, spin: estimate.spin, cd: estimate.cd, wind: state.wind, density: state.density, spinDecay: state.spinDecay, gust: state.gust, physicsMode: state.physicsMode, axisTilt: state.axisTilt, axisAzimuth: state.axisAzimuth, lateralWind: state.lateralWind }, "full");
    const error = replayError(points, replay.points);
    const visibleReplay = directFit ? replayDuringObservation(points, replay.points) : replay.points;
    state.aiResult = { observed: points, estimate, replay: visibleReplay, source, directFit };
    $("aiSpeed").textContent = `${estimate.speed.toFixed(2)} m/s`;
    $("aiAngle").textContent = `${estimate.angle.toFixed(1)}°`;
    $("aiCd").textContent = estimate.cd.toFixed(3);
    $("aiSpin").textContent = formatSpin(estimate.spin);
    $("aiConfidenceLabel").textContent = directFit ? "拟合质量" : "近邻相似度";
    $("aiConfidence").textContent = `${Math.round(estimate.confidence * 100)}%`;
    $("aiError").textContent = `${error.toFixed(2)} m`;
    $("aiMessage").textContent = directFit
      ? "反演完成，请在右侧回放图中对照输入轨迹与 AI 参数回放。"
      : `已从${source}提取 ${points.length} 个点，使用 9 个近邻样本完成反演；匹配度是相似度指标，不是概率置信区间。`;
    $("fitNote").textContent = directFit
      ? `已从${source}读取 ${points.length} 个飞行点，以当前球种、风场与空气设置为前提，对最多 80 个均匀抽样点做二维动力学拟合；RMSE 使用全部输入点计算。`
      : "当前轨迹采用物理样本近邻反演，右侧蓝线为 AI 参数回放。";
    drawReplay(points, visibleReplay);
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
    const uniquePoints = points.filter((point, index) => index === 0 || point.t > points[index - 1].t);
    const normalized = [{ t: 0, x: 0, y: 0 }];
    for (let index = 1; index < uniquePoints.length; index += 1) {
      const point = uniquePoints[index];
      const height = point.y - first.y;
      if (height <= 0) break;
      normalized.push({
        t: point.t - first.t,
        x: point.x - first.x,
        y: height
      });
    }
    return normalized;
  }

  function generatedDemoCSV() {
    const trajectory = simulate({
      ball: "tennis", speed: 22, angle: 45, spin: 1200, cd: BALLS.tennis.cd,
      wind: 0, density: 1.225, spinDecay: 0.4, gust: 0, physicsMode: "advanced"
    }, "full");
    const lastFlightIndex = trajectory.points.findLastIndex((point) => point.y > 0);
    const sampled = trajectory.points.filter((point, index) => point.y > 0 && (index % 10 === 0 || index === lastFlightIndex));
    sampled.unshift(trajectory.points[0]);
    return ["time_s,x_m,y_m", ...sampled.map((point) => `${point.t.toFixed(3)},${point.x.toFixed(6)},${point.y.toFixed(6)}`)].join("\n");
  }

  async function useDemoData() {
    const requestId = ++state.observedRequestId;
    state.observedSource = "demo";
    setAIInputSource("demo");
    $("uploadName").textContent = "正在载入示范数据…";
    const text = generatedDemoCSV();
    const points = parseCSV(text);
    if (requestId !== state.observedRequestId) return;
    state.observed = points;
    state.observedSource = "demo";
    state.aiResult = null;
    setAIInputSource("demo");
    $("uploadName").textContent = "demo_observation.csv · 示范数据";
    queueAI(points, "示范数据", requestId);
  }

  function selectUploadData() {
    $("csvInput").click();
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
    $("spaceMode").addEventListener("change", (event) => { state.spaceMode = event.target.value; render(); });
    $("speedRange").addEventListener("input", (event) => { state.speed = Number(event.target.value); updateLabels(); render(); });
    $("angleRange").addEventListener("input", (event) => { state.angle = Number(event.target.value); updateLabels(); render(); });
    $("spinRange").addEventListener("input", (event) => { state.spin = Number(event.target.value); updateLabels(); render(); });
    $("windRange").addEventListener("input", (event) => { state.wind = Number(event.target.value); updateLabels(); render(); });
    $("physicsMode").addEventListener("change", (event) => { state.physicsMode = event.target.value; render(); });
    $("densityRange").addEventListener("input", (event) => { state.density = Number(event.target.value); updateLabels(); render(); });
    $("spinDecayRange").addEventListener("input", (event) => { state.spinDecay = Number(event.target.value); updateLabels(); render(); });
    $("gustRange").addEventListener("input", (event) => { state.gust = Number(event.target.value); updateLabels(); render(); });
    $("axisTiltRange").addEventListener("input", (event) => { state.axisTilt = Number(event.target.value); updateLabels(); render(); });
    $("axisAzimuthRange").addEventListener("input", (event) => { state.axisAzimuth = Number(event.target.value); updateLabels(); render(); });
    $("lateralWindRange").addEventListener("input", (event) => { state.lateralWind = Number(event.target.value); updateLabels(); render(); });
    $("resetButton").addEventListener("click", () => {
      state.ball = "tennis"; state.speed = 22; state.angle = 45; state.spin = 1200; state.wind = 0; state.density = 1.225; state.spinDecay = 0.4; state.gust = 0; state.physicsMode = "advanced"; state.spaceMode = "3d"; state.axisTilt = 35; state.axisAzimuth = 90; state.lateralWind = 0;
      state.observedRequestId += 1;
      state.observed = null;
      state.observedSource = "current";
      state.aiResult = null;
      setAIInputSource("current");
      $("uploadName").textContent = "当前模拟轨迹 · 实时参数";
      $("ballType").value = state.ball; $("spaceMode").value = state.spaceMode; $("speedRange").value = state.speed; $("angleRange").value = state.angle; $("spinRange").value = state.spin; $("windRange").value = state.wind; $("densityRange").value = state.density; $("spinDecayRange").value = state.spinDecay; $("gustRange").value = state.gust; $("axisTiltRange").value = state.axisTilt; $("axisAzimuthRange").value = state.axisAzimuth; $("lateralWindRange").value = state.lateralWind; $("physicsMode").value = state.physicsMode;
      document.querySelectorAll(".case-button").forEach((button) => button.classList.toggle("active", button.dataset.case === "baseline"));
      render();
    });
    $("animateButton").addEventListener("click", startAnimation);
    $("heroStart").addEventListener("click", () => $("simulator").scrollIntoView({ behavior: "smooth" }));
    $("aiRunButton").addEventListener("click", () => {
      const requestId = ++state.observedRequestId;
      state.observed = state.current.full.points;
      state.observedSource = "current";
      state.aiResult = null;
      setAIInputSource("current");
      $("uploadName").textContent = "当前模拟轨迹 · 实时参数";
      queueAI(state.observed, "当前模拟轨迹", requestId);
    });
    $("useUploadDataButton").addEventListener("click", selectUploadData);
    $("useDemoButton").addEventListener("click", useDemoData);
    $("csvInput").addEventListener("change", (event) => {
      const file = event.target.files && event.target.files[0];
      if (!file) return;
      const requestId = ++state.observedRequestId;
      state.observedSource = "upload";
      state.aiResult = null;
      setAIInputSource("upload");
      const reader = new FileReader();
      reader.onload = () => {
        if (requestId !== state.observedRequestId) return;
        const points = parseCSV(String(reader.result));
        state.observed = points;
        state.observedSource = "upload";
        state.aiResult = null;
        $("uploadName").textContent = `${file.name} · ${points.length} 个点`;
        if (points.length < 8) {
          $("aiMessage").textContent = "CSV 中至少需要 8 个有效点，格式为 time_s,x_m,y_m。";
          updateStatus("轨迹数据不足");
          return;
        }
        queueAI(points, `上传文件 ${file.name}`, requestId);
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
    window.addEventListener("resize", () => {
      if (!state.current) return;
      drawTrajectory();
      if (state.observedSource !== "current" && state.observed) {
        if (state.aiResult && state.aiResult.observed === state.observed) drawReplay(state.observed, state.aiResult.replay);
        else runAI(state.observed, state.observedSource === "demo" ? "示范数据" : "上传数据");
      }
    });
    setupNavSpy();
  }

  bindControls();
  render();
})();

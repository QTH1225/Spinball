const fs = require("fs");
const vm = require("vm");

const makeContext = () => new Proxy({}, {
  get(target, key) {
    if (key === "measureText") return (text) => ({ width: String(text).length * 8 });
    if (!(key in target)) target[key] = () => {};
    return target[key];
  },
  set(target, key, value) {
    target[key] = value;
    return true;
  }
});

const elements = new Map();
const makeElement = (id) => {
  const element = {
    id,
    value: "",
    textContent: "",
    innerHTML: "",
    dataset: {},
    classList: { add() {}, remove() {}, toggle() {} },
    setAttribute() {},
    addEventListener() {},
    getBoundingClientRect: () => ({ width: 800, height: 500 }),
    scrollIntoView() {}
  };
  if (id === "trajectoryCanvas" || id === "aiReplayCanvas") element.getContext = () => makeContext();
  return element;
};

global.document = {
  getElementById(id) {
    if (!elements.has(id)) elements.set(id, makeElement(id));
    return elements.get(id);
  },
  querySelector() { return null; },
  querySelectorAll() { return []; }
};
global.window = {
  devicePixelRatio: 1,
  innerHeight: 900,
  addEventListener() {},
  setTimeout(callback) { callback(); },
  requestAnimationFrame() { return 1; }
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};

const sourcePath = "code/app.js";
const source = fs.readFileSync(sourcePath, "utf8");
const marker = "  bindControls();\n  render();\n})();";
const instrumented = source.replace(marker, "  globalThis.__internals = { state, simulate, fitTrajectory, fitObjective, optimizeFitStart, parseCSV, generatedDemoCSV };\n  bindControls();\n  render();\n})();");
vm.runInThisContext(instrumented, { filename: sourcePath });

const { state, simulate, fitTrajectory, fitObjective, optimizeFitStart, parseCSV, generatedDemoCSV } = global.__internals;
const known = { speed: 24, angle: 38, spin: 1500, cd: 0.62 };
Object.assign(state, { ball: "tennis", ...known, wind: 0, density: 1.225, spinDecay: 0.4, gust: 0, physicsMode: "advanced" });

const generated = simulate({ ...known, ball: state.ball, dt: state.dt }, "full");
const csv = ["time_s,x_m,y_m"];
generated.points.forEach((point, index) => {
  const x = point.x + 0.00035 * Math.sin(index * 0.71);
  const y = point.y + 0.00035 * Math.cos(index * 0.53);
  csv.push(`${point.t.toFixed(6)},${x.toFixed(6)},${y.toFixed(6)}`);
});
const outputPath = "data/test_fit_observation.csv";
fs.writeFileSync(outputPath, `${csv.join("\n")}\n`, "utf8");

const parsed = parseCSV(csv.join("\n"));
const fitted = fitTrajectory(parsed);
Object.assign(state, { speed: 22, angle: 45, spin: 1200, wind: 0, density: 1.225, spinDecay: 0.4, gust: 0, physicsMode: "advanced" });
const demoText = generatedDemoCSV();
fs.writeFileSync("data/demo_observation.csv", `${demoText}\n`, "utf8");
const demoParsed = parseCSV(demoText);
const demoFit = fitTrajectory(demoParsed);
const knownObjective = fitObjective(parsed, known, state.dt / 2);
const fittedObjective = fitObjective(parsed, fitted, state.dt / 2);
const knownStartFit = optimizeFitStart(known, parsed, state.dt / 2, {
  speed: { min: 8, max: 36 }, angle: { min: 5, max: 75 }, spin: { min: -2600, max: 2600 }, cd: { min: 0.12, max: 0.8 }
});
const bounds = {
  speed: { min: 8, max: 36 }, angle: { min: 5, max: 75 }, spin: { min: -2600, max: 2600 }, cd: { min: 0.12, max: 0.8 }
};
const startFits = [
  { speed: 26, angle: 30, spin: 1300, cd: 0.62 },
  { speed: 26, angle: 40, spin: 1300, cd: 0.62 },
  { speed: 20, angle: 40, spin: 1300, cd: 0.62 },
  { speed: 28, angle: 50, spin: 1300, cd: 0.42 }
].map((start) => {
  const result = optimizeFitStart(start, parsed, state.dt / 2, bounds);
  return { start, result: { speed: result.speed, angle: result.angle, spin: result.spin, cd: result.cd, rmse_m: result.rmse } };
});
const nearby = [
  { speed: 24, angle: 38, spin: 1500, cd: 0.62 },
  { speed: 24, angle: 38, spin: 1500, cd: 0.8 },
  { speed: 25, angle: 38, spin: 1500, cd: 0.62 },
  { speed: 24, angle: 40, spin: 1500, cd: 0.62 },
  { speed: 24, angle: 38, spin: 0, cd: 0.62 },
  { speed: 24, angle: 38, spin: 2600, cd: 0.62 }
].map((candidate) => ({ candidate, rmse_m: Number(fitObjective(parsed, candidate, state.dt / 2).rmse.toFixed(6)) }));
console.log(JSON.stringify({
  data_file: outputPath,
  points: generated.points.length,
  known,
  fitted: {
    speed: Number(fitted.speed.toFixed(3)),
    angle: Number(fitted.angle.toFixed(3)),
    spin: Number(fitted.spin.toFixed(3)),
    cd: Number(fitted.cd.toFixed(4)),
    rmse_m: Number(fitted.rmse.toFixed(6)),
    confidence: Number(fitted.confidence.toFixed(4))
  },
  demo_check: { points_after_ground_cut: demoParsed.length, first_point: demoParsed[0], last_point: demoParsed[demoParsed.length - 1], fitted: { speed: Number(demoFit.speed.toFixed(3)), angle: Number(demoFit.angle.toFixed(3)), spin: Number(demoFit.spin.toFixed(3)), cd: Number(demoFit.cd.toFixed(4)), rmse_m: Number(demoFit.rmse.toFixed(6)) } },
  objective_check: {
    known_rmse_m: Number(knownObjective.rmse.toFixed(6)),
    fitted_rmse_m: Number(fittedObjective.rmse.toFixed(6)),
    known_start_fit: { speed: Number(knownStartFit.speed.toFixed(3)), angle: Number(knownStartFit.angle.toFixed(3)), spin: Number(knownStartFit.spin.toFixed(3)), cd: Number(knownStartFit.cd.toFixed(4)), rmse_m: Number(knownStartFit.rmse.toFixed(6)) },
    start_fits: startFits.map((item) => ({ start: item.start, result: { speed: Number(item.result.speed.toFixed(3)), angle: Number(item.result.angle.toFixed(3)), spin: Number(item.result.spin.toFixed(3)), cd: Number(item.result.cd.toFixed(4)), rmse_m: Number(item.result.rmse_m.toFixed(6)) } })),
    nearby
  }
}, null, 2));


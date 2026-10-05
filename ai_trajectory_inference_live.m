%% AI + Mechanics: uploaded trajectory inference
% This script mirrors the web 02 / AI INFERENCE upload workflow.
% CSV input requires at least three columns: time_s, x_m, y_m.
% Estimated parameters: initial speed, launch angle, spin rate, and Cd.
% At most 80 uniformly sampled points are used during fitting.
% Final RMSE is calculated on all cleaned input points.
% MATLAB R2020b or newer; no Optimization Toolbox required.

clear; close all; clc;

%% 1. Input file and current physical settings
projectRoot = fileparts(mfilename('fullpath'));
inputFile = fullfile(projectRoot, 'data', 'test_fit_observation.csv');
if ~isfile(inputFile)
    [fileName, filePath] = uigetfile({'*.csv', 'CSV trajectory (*.csv)'}, ...
        'Select trajectory CSV');
    if isequal(fileName, 0)
        error('No input file was selected.');
    end
    inputFile = fullfile(filePath, fileName);
end

p = defaultParameters();
inferenceMode = 'upload_fit';
modeOverride = getenv('AI_INFERENCE_MODE');
if any(strcmpi(modeOverride, {'upload_fit', 'knn'}))
    inferenceMode = lower(modeOverride);
end
fprintf('AI + mechanics trajectory inference\n');
fprintf('Input: %s\n', inputFile);
fprintf('Ball: %s | wind: %.2f m/s | density: %.3f kg/m^3\n', ...
    p.ballName, p.wind, p.density);
fprintf('Mode: %s\n', inferenceMode);
formulaText = { ...
    'v_rel = v - w'; ...
    'rho(y) = rho0 * exp[-g*max(y,0)/(R*T)]'; ...
    'Re = rho*|v_rel|*D/mu'; ...
    'S = omega*R/|v_rel|'; ...
    'q = 0.5*rho*|v_rel|^2'; ...
    'F_D = -q*Cd*A*v_rel/|v_rel|'; ...
    'Cl = clamp(sign(S)*1.55*|S|/(1+1.8*|S|), -0.72, 0.72)'; ...
    'F_M = q*Cl*A*[-v_rel_y, v_rel_x]/|v_rel|'; ...
    'm*a = [0,-m*g] + F_D + F_B + F_M'; ...
    's(n+1) = s(n) + dt*(k1 + 2*k2 + 2*k3 + k4)/6'};
disp('Core equations:');
disp(formulaText);

%% 2. Physical equations used by the inverse model
% Relative velocity: v_rel = v - w.
% Advanced density: rho(y) = rho0 * exp[-g*max(y,0)/(R*T)].
% Reynolds number: Re = rho*|v_rel|*D/mu.
% Spin parameter: S = omega*R/|v_rel|.
% Dynamic pressure: q = 1/2*rho*|v_rel|^2; area A = pi*R^2.
% Drag: F_D = -q*Cd*A*v_rel/|v_rel|.
% Lift coefficient: Cl = clamp(sign(S)*1.55*|S|/(1+1.8*|S|), -0.72, 0.72).
% 2-D Magnus force: F_M = q*Cl*A*[-v_rel_y, v_rel_x]/|v_rel|.
% Equation of motion: m*a = [0,-m*g] + F_D + F_B + F_M.
% Numerical integration: fourth-order Runge-Kutta.

%% 3. Read, translate to the origin, and keep the free-flight segment
observed = loadTrajectory(inputFile);
if height(observed) < 8
    error('Fewer than 8 valid flight points remain after cleaning.');
end
fprintf('Cleaned free-flight points: %d\n', height(observed));
fprintf('Observation time: %.4f to %.4f s\n', observed.t(1), observed.t(end));

%% 4. Uniform sampling for fitting
sampled = sampleTrajectory(observed, 80);
timeGaps = diff(sampled.t);
timeGaps = sort(timeGaps(timeGaps > 1e-5));
if isempty(timeGaps)
    medianGap = p.dt;
else
    medianGap = timeGaps(ceil(numel(timeGaps) / 2));
end
integrationStep = min(p.dt, medianGap / 2);
integrationStep = min(max(integrationStep, 0.002), 0.01);
fprintf('Fit points: %d; integration step: %.5f s\n', ...
    height(sampled), integrationStep);

%% 5. Inversion: direct dynamics fit or browser-style KNN
if strcmpi(inferenceMode, 'knn')
    best = inferKnn(observed, p);
    integrationStep = p.dt;
else
    bounds = [8, 36; 5, 75; -2600, 2600; 0.12, 0.80];
    startCandidates = makeStartCandidates(sampled, p, bounds);
    startResults = repmat(emptyFitResult(), numel(startCandidates), 1);
    for k = 1:numel(startCandidates)
        startResults(k) = evaluateCandidate(startCandidates(k), sampled, p, ...
            integrationStep, bounds);
    end

    [~, order] = sort([startResults.score]);
    topStarts = startResults(order(1:min(5, numel(order))));
    optimized = repmat(emptyFitResult(), numel(topStarts), 1);
    for k = 1:numel(topStarts)
        optimized(k) = optimizeNelderMead(topStarts(k).candidate, sampled, p, ...
            integrationStep, bounds);
    end

    [~, bestIndex] = min([optimized.score]);
    best = optimized(bestIndex);
end
fullReplay = simulate2D(best.candidate, p, integrationStep);
fullRmse = trajectoryRmse(observed, fullReplay);
if isfield(best, 'confidence')
    confidence = best.confidence;
else
    confidenceScale = max(0.02, max(hypot(observed.x, observed.y)) * 0.025);
    confidence = min(max(exp(-fullRmse / confidenceScale), 0), 0.99);
end

%% 6. Print inference result
resultTable = table( ...
    best.candidate.speed, best.candidate.angle, best.candidate.spin, ...
    best.candidate.cd, fullRmse, confidence, ...
    'VariableNames', {'speed_mps', 'angle_deg', 'spin_rpm', 'Cd', ...
    'RMSE_m', 'confidence'});
disp(resultTable);
fprintf('Objective score: %.6f\n', best.score);
fprintf('Sampled RMSE: %.6f m; all-point RMSE: %.6f m\n', ...
    best.rmse, fullRmse);

%% 7. Input trajectory versus AI replay
prediction = interpolateTrajectory(fullReplay, observed.t);
figure('Color', 'w', 'Name', 'AI 轨迹参数反演');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(observed.x, observed.y, '.', 'Color', [0.95, 0.35, 0.20], ...
    'MarkerSize', 12, 'DisplayName', '输入轨迹');
hold on;
plot(fullReplay.x, fullReplay.y, '-', 'Color', [0.10, 0.35, 0.95], ...
    'LineWidth', 2.0, 'DisplayName', 'AI 参数回放');
scatter(observed.x(1), observed.y(1), 42, [0.07, 0.24, 0.19], ...
    'filled', 'DisplayName', '起点');
grid on; box on;
xlabel('水平距离 x / m'); ylabel('高度 y / m');
title(sprintf('轨迹回放：RMSE = %.4f m', fullRmse));
legend('Location', 'best');

nexttile;
residual = hypot(observed.x - prediction.x, observed.y - prediction.y);
plot(observed.t, residual, '-', 'Color', [0.07, 0.24, 0.19], ...
    'LineWidth', 1.6, 'DisplayName', '逐点位置误差');
hold on;
yline(fullRmse, '--', 'Color', [0.95, 0.49, 0.20], ...
    'LineWidth', 1.2, 'DisplayName', 'RMSE');
grid on; box on;
xlabel('时间 t / s'); ylabel('位置误差 / m');
title('全部输入点的残差');
legend('Location', 'best');

%% 8. Save reproducible result
save(fullfile(projectRoot, 'ai_inference_result.mat'), ...
    'inputFile', 'p', 'observed', 'sampled', 'best', 'fullReplay', ...
    'prediction', 'fullRmse', 'confidence');
fprintf('Saved result: %s\n', ...
    fullfile(projectRoot, 'ai_inference_result.mat'));

%% Local functions
function p = defaultParameters()
    p = struct();
    p.ballName = 'tennis ball';
    p.mass = 0.057;
    p.radius = 0.0335;
    p.diameter = 2 * p.radius;
    p.area = pi * p.radius^2;
    p.cd = 0.62;
    p.speed = 22;
    p.angle = 45;
    p.spin = 1200;
    p.wind = 0;
    p.density = 1.225;
    p.spinDecay = 0.40;
    p.gust = 0;
    p.physicsMode = 'advanced';
    p.g = 9.81;
    p.airViscosity = 1.81e-5;
    p.gasConstant = 287.05;
    p.airTemperature = 288.15;
    p.dt = 0.008;
    p.maxTime = 8;
end

function observed = loadTrajectory(inputFile)
    raw = readtable(inputFile, 'VariableNamingRule', 'preserve');
    names = string(raw.Properties.VariableNames);
    timeName = findColumn(names, ["time_s", "time", "t"]);
    xName = findColumn(names, ["x_m", "x", "X"]);
    yName = findColumn(names, ["y_m", "y", "Y", "z_m", "z", "Z"]);
    if isempty(timeName) || isempty(xName) || isempty(yName)
        if width(raw) < 3
            error('CSV must contain at least time, x, and y columns.');
        end
        values = table2array(raw(:, 1:3));
    else
        values = [raw.(timeName), raw.(xName), raw.(yName)];
    end
    values = double(values);
    values = values(all(isfinite(values), 2), :);
    values = sortrows(values, 1);
    values = values([true; diff(values(:, 1)) > 0], :);
    if size(values, 1) < 2
        error('The CSV does not contain enough valid points.');
    end

    first = values(1, :);
    normalized = [0, 0, 0];
    for k = 2:size(values, 1)
        height = values(k, 3) - first(3);
        if height <= 0
            break;
        end
        normalized(end + 1, :) = [values(k, 1) - first(1), ...
            values(k, 2) - first(2), height]; %#ok<AGROW>
    end
    observed = array2table(normalized, ...
        'VariableNames', {'t', 'x', 'y'});
end

function name = findColumn(names, aliases)
    name = '';
    normalized = lower(regexprep(names, '[^a-zA-Z0-9]', ''));
    for k = 1:numel(aliases)
        alias = lower(regexprep(aliases(k), '[^a-zA-Z0-9]', ''));
        index = find(normalized == alias, 1);
        if ~isempty(index)
            name = char(names(index));
            return;
        end
    end
end

function sampled = sampleTrajectory(points, maximumPoints)
    if height(points) <= maximumPoints
        sampled = points;
        return;
    end
    indices = round(linspace(1, height(points), maximumPoints));
    sampled = points(indices, :);
end

function candidates = makeStartCandidates(points, p, bounds)
    speeds = [10, 18, 26, 34];
    angles = [10, 30, 50, 70];
    spins = [-2600, -1300, 0, 1300, 2600];
    cds = [0.18, 0.38, 0.58, 0.78];
    candidates = repmat(struct('speed', 0, 'angle', 0, ...
        'spin', 0, 'cd', 0), 0, 1);
    for speed = speeds
        for angle = angles
            for spin = spins
                for cd = cds
                    candidates(end + 1, 1) = struct('speed', speed, ...
                        'angle', angle, 'spin', spin, 'cd', cd); %#ok<AGROW>
                end
            end
        end
    end

    firstIndex = min(3, height(points));
    startDt = max(points.t(firstIndex) - points.t(1), 1e-5);
    startVx = (points.x(firstIndex) - points.x(1)) / startDt;
    startVy = (points.y(firstIndex) - points.y(1)) / startDt;
    speedGuess = clampValue(hypot(startVx, startVy) * 1.12, ...
        bounds(1, 1), bounds(1, 2));
    angleGuess = clampValue(atan2d(startVy, max(startVx, 1e-5)), ...
        bounds(2, 1), bounds(2, 2));
    candidates(end + 1, 1) = struct('speed', speedGuess, ...
        'angle', angleGuess, 'spin', p.spin, 'cd', p.cd);
    candidates(end + 1, 1) = struct('speed', 20, ...
        'angle', 40, 'spin', 1200, 'cd', p.cd);
    candidates(end + 1, 1) = struct('speed', 28, ...
        'angle', 52, 'spin', -1200, 'cd', 0.42);
end

function result = emptyFitResult()
    result = struct('candidate', struct('speed', 0, 'angle', 0, ...
        'spin', 0, 'cd', 0), 'score', inf, 'rmse', inf, ...
        'replay', []);
end

function result = evaluateCandidate(candidate, points, p, integrationStep, bounds)
    candidate = clampCandidate(candidate, bounds);
    replay = simulate2D(candidate, p, integrationStep);
    prediction = interpolateTrajectory(replay, points.t);
    positionError = hypot(points.x - prediction.x, points.y - prediction.y);
    weights = ones(height(points), 1);
    weights(end) = 1.35;
    rmse = sqrt(sum(weights .* positionError.^2) / height(points));
    durationPenalty = 0;
    if replay.t(end) + integrationStep < points.t(end)
        durationPenalty = 0.65 * (points.t(end) - replay.t(end));
    end
    result = struct('candidate', candidate, 'score', ...
        rmse + durationPenalty, 'rmse', rmse, 'replay', replay);
end

function result = optimizeNelderMead(start, points, p, integrationStep, bounds)
    initial = clampCandidate(start, bounds);
    initialVector = candidateVector(initial);
    steps = [4, 8, 800, 0.16];
    simplex = zeros(5, 4);
    evaluations = repmat(emptyFitResult(), 5, 1);
    simplex(1, :) = initialVector;
    evaluations(1) = evaluateCandidate(initial, points, p, ...
        integrationStep, bounds);
    for k = 1:4
        simplex(k + 1, :) = initialVector;
        simplex(k + 1, k) = simplex(k + 1, k) + steps(k);
        evaluations(k + 1) = evaluateCandidate( ...
            vectorCandidate(simplex(k + 1, :)), points, p, ...
            integrationStep, bounds);
        simplex(k + 1, :) = candidateVector(evaluations(k + 1).candidate);
    end

    for iteration = 1:72 %#ok<NASGU>
        scores = [evaluations.score].';
        [~, order] = sort(scores);
        simplex = simplex(order, :);
        evaluations = evaluations(order);
        best = evaluations(1);
        centroid = mean(simplex(1:4, :), 1);
        reflected = evaluateCandidate( ...
            vectorCandidate(2 * centroid - simplex(5, :)), points, p, ...
            integrationStep, bounds);
        if reflected.score < best.score
            expanded = evaluateCandidate( ...
                vectorCandidate(centroid + 2 * ...
                (candidateVector(reflected.candidate) - centroid)), ...
                points, p, integrationStep, bounds);
            if expanded.score < reflected.score
                evaluations(5) = expanded;
            else
                evaluations(5) = reflected;
            end
        elseif reflected.score < evaluations(4).score
            evaluations(5) = reflected;
        else
            contracted = evaluateCandidate( ...
                vectorCandidate(centroid + 0.5 * ...
                (simplex(5, :) - centroid)), points, p, ...
                integrationStep, bounds);
            if contracted.score < evaluations(5).score
                evaluations(5) = contracted;
            else
                for k = 2:5
                    shrunk = candidateVector(best.candidate) + ...
                        0.5 * (simplex(k, :) - candidateVector(best.candidate));
                    evaluations(k) = evaluateCandidate( ...
                        vectorCandidate(shrunk), points, p, ...
                        integrationStep, bounds);
                end
            end
        end
        for k = 1:5
            simplex(k, :) = candidateVector(evaluations(k).candidate);
        end
        spread = max(abs([evaluations.score] - min([evaluations.score])));
        if spread < 1e-5
            break;
        end
    end
    [~, bestIndex] = min([evaluations.score]);
    result = evaluations(bestIndex);
end

function candidate = clampCandidate(candidate, bounds)
    values = candidateVector(candidate);
    values = min(max(values, bounds(:, 1).'), bounds(:, 2).');
    candidate = vectorCandidate(values);
end

function values = candidateVector(candidate)
    values = [candidate.speed, candidate.angle, candidate.spin, candidate.cd];
end

function candidate = vectorCandidate(values)
    candidate = struct('speed', values(1), 'angle', values(2), ...
        'spin', values(3), 'cd', values(4));
end

function result = inferKnn(points, p)
    training = makeTrainingSet(p);
    target = trajectoryFeatures(points);
    featureMatrix = zeros(numel(training), numel(target));
    for k = 1:numel(training)
        featureMatrix(k, :) = training(k).vector;
    end
    minimum = min(featureMatrix, [], 1);
    maximum = max(featureMatrix, [], 1);
    span = max(maximum - minimum, 1e-6);
    distances = sqrt(sum(((featureMatrix - target) ./ span).^2, 2));
    [sortedDistances, order] = sort(distances);
    neighborCount = min(9, numel(order));
    selected = order(1:neighborCount);
    weights = 1 ./ (0.02 + sortedDistances(1:neighborCount));
    weights = weights / sum(weights);
    parameterMatrix = zeros(neighborCount, 4);
    for k = 1:neighborCount
        parameterMatrix(k, :) = [training(selected(k)).speed, ...
            training(selected(k)).angle, training(selected(k)).spin, ...
            training(selected(k)).cd];
    end
    estimate = sum(parameterMatrix .* weights, 1);
    meanDistance = mean(sortedDistances(1:neighborCount));
    result = emptyFitResult();
    result.candidate = vectorCandidate(estimate);
    result.score = meanDistance;
    result.replay = simulate2D(result.candidate, p, p.dt);
    result.rmse = trajectoryRmse(points, result.replay);
    result.confidence = min(max(exp(-meanDistance), 0), 0.99);
    result.method = 'knn';
end

function training = makeTrainingSet(p)
    speeds = [14, 18, 22, 26, 30];
    angles = [10, 18, 26, 34, 42];
    spins = [-2200, -1100, 0, 1100, 2200];
    cds = [0.18, 0.28, 0.42, 0.56, 0.68];
    training = repmat(struct('vector', [], 'speed', 0, ...
        'angle', 0, 'spin', 0, 'cd', 0), 0, 1);
    for speed = speeds
        for angle = angles
            for spin = spins
                for cd = cds
                    candidate = struct('speed', speed, 'angle', angle, ...
                        'spin', spin, 'cd', cd);
                    replay = simulate2D(candidate, p, p.dt);
                    vector = trajectoryFeatures(replay);
                    training(end + 1, 1) = struct('vector', vector, ...
                        'speed', speed, 'angle', angle, 'spin', spin, ...
                        'cd', candidate.cd); %#ok<AGROW>
                end
            end
        end
    end
end

function vector = trajectoryFeatures(points)
    firstIndex = 1;
    if istable(points)
        lastIndex = height(points);
    else
        lastIndex = numel(points.t);
    end
    pointCount = lastIndex;
    duration = max(points.t(lastIndex) - points.t(firstIndex), 1e-5);
    firstGapIndex = min(3, lastIndex);
    startDt = max(points.t(firstGapIndex) - points.t(firstIndex), 1e-5);
    startVx = (points.x(firstGapIndex) - points.x(firstIndex)) / startDt;
    startVy = (points.y(firstGapIndex) - points.y(firstIndex)) / startDt;
    endGapIndex = max(1, lastIndex - 2);
    endDt = max(points.t(lastIndex) - points.t(endGapIndex), 1e-5);
    endVx = (points.x(lastIndex) - points.x(endGapIndex)) / endDt;
    endVy = (points.y(lastIndex) - points.y(endGapIndex)) / endDt;
    maxHeight = max(points.y);
    firstAngle = atan2d(startVy, startVx);
    endSpeed = hypot(endVx, endVy);
    curvature = 0;
    for k = 3:pointCount
        firstVector = [points.x(k - 1) - points.x(k - 2), ...
            points.y(k - 1) - points.y(k - 2)];
        secondVector = [points.x(k) - points.x(k - 1), ...
            points.y(k) - points.y(k - 1)];
        chord = hypot(points.x(k) - points.x(k - 2), ...
            points.y(k) - points.y(k - 2));
        denominator = norm(firstVector) * norm(secondVector) * chord;
        if denominator > 1e-9
            cross2D = firstVector(1) * secondVector(2) - ...
                firstVector(2) * secondVector(1);
            curvature = max(curvature, 2 * abs(cross2D) / denominator);
        end
    end
    vector = [points.x(lastIndex) - points.x(firstIndex), ...
        maxHeight - points.y(firstIndex), duration, endSpeed, ...
        firstAngle, curvature];
end

function replay = simulate2D(candidate, p, integrationStep)
    angle = deg2rad(candidate.angle);
    state = [0, 0, candidate.speed * cos(angle), ...
        candidate.speed * sin(angle)];
    trajectory = [0, state];
    initialAero = aerodynamicState2D(state, candidate, p, 0);
    t = 0;
    maxSteps = ceil(p.maxTime / integrationStep);
    for step = 1:maxSteps %#ok<NASGU>
        previous = state;
        state = rk4Step2D(state, candidate, p, integrationStep, t);
        t = t + integrationStep;
        if state(2) < 0 && t > integrationStep
            fraction = previous(2) / max(previous(2) - state(2), 1e-12);
            landing = previous + fraction * (state - previous);
            landing(2) = 0;
            landingTime = t - integrationStep + integrationStep * fraction;
            trajectory(end + 1, :) = [landingTime, landing]; %#ok<AGROW>
            break;
        end
        state(2) = max(0, state(2));
        trajectory(end + 1, :) = [t, state]; %#ok<AGROW>
    end
    replay = struct();
    replay.t = trajectory(:, 1);
    replay.x = trajectory(:, 2);
    replay.y = trajectory(:, 3);
    replay.vx = trajectory(:, 4);
    replay.vy = trajectory(:, 5);
    replay.initialAero = initialAero;
end

function nextState = rk4Step2D(state, candidate, p, dt, t)
    k1 = derivative2D(state, candidate, p, t);
    k2 = derivative2D(state + dt * k1 / 2, candidate, p, t + dt / 2);
    k3 = derivative2D(state + dt * k2 / 2, candidate, p, t + dt / 2);
    k4 = derivative2D(state + dt * k3, candidate, p, t + dt);
    nextState = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
end

function derivative = derivative2D(state, candidate, p, t)
    aero = aerodynamicState2D(state, candidate, p, t);
    acceleration = [0, -p.g];
    if aero.relativeSpeed > 1e-8
        dragAcceleration = -0.5 * aero.density * aero.cd * p.area * ...
            aero.relativeSpeed * [aero.relativeVx, aero.relativeVy] / p.mass;
        acceleration = acceleration + dragAcceleration;
        acceleration(2) = acceleration(2) + aero.buoyancyForce / p.mass;
        magnusFactor = 0.5 * aero.density * p.area * aero.cl * ...
            aero.relativeSpeed^2 / p.mass;
        normal = [-aero.relativeVy, aero.relativeVx] / aero.relativeSpeed;
        acceleration = acceleration + magnusFactor * normal;
    end
    derivative = [state(3), state(4), acceleration(1), acceleration(2)];
end

function aero = aerodynamicState2D(state, candidate, p, t)
    y = state(2);
    if strcmpi(p.physicsMode, 'advanced')
        density = p.density * exp(-p.g * max(y, 0) / ...
            (p.gasConstant * p.airTemperature));
    else
        density = p.density;
    end
    gust = 0;
    shear = 0;
    if strcmpi(p.physicsMode, 'advanced')
        gust = p.gust * sin(2 * pi * 0.65 * t + 0.12 * max(y, 0));
        shear = 0.025 * p.wind * max(y, 0);
    end
    wind = p.wind + gust + shear;
    relativeVx = state(3) - wind;
    relativeVy = state(4);
    relativeSpeed = hypot(relativeVx, relativeVy);
    if strcmpi(p.physicsMode, 'advanced')
        spinRad = candidate.spin * 2 * pi / 60 * exp(-p.spinDecay * t);
    else
        spinRad = candidate.spin * 2 * pi / 60;
    end
    spinParameter = 0;
    if relativeSpeed > 1e-5
        spinParameter = spinRad * p.radius / relativeSpeed;
    end
    reynolds = density * relativeSpeed * p.diameter / p.airViscosity;
    if isfield(p, 'smooth') && p.smooth
        cd = smoothSphereCd(reynolds);
    else
        cd = candidate.cd;
    end
    if strcmpi(p.physicsMode, 'advanced')
        cl = sign(spinParameter) * (1.55 * abs(spinParameter)) / ...
            (1 + 1.8 * abs(spinParameter));
        cl = clampValue(cl, -0.72, 0.72);
    else
        cl = clampValue(0.000115 * candidate.spin, -0.34, 0.34);
    end
    dynamicPressure = 0.5 * density * relativeSpeed^2;
    displacedVolume = 4 / 3 * pi * p.radius^3;
    buoyancyForce = density * displacedVolume * p.g;
    aero = struct('density', density, 'relativeVx', relativeVx, ...
        'relativeVy', relativeVy, 'relativeSpeed', relativeSpeed, ...
        'spinRad', spinRad, 'spinParameter', spinParameter, ...
        'reynolds', reynolds, 'cd', cd, 'cl', cl, ...
        'buoyancyForce', buoyancyForce, ...
        'dragForce', dynamicPressure * p.area * cd, ...
        'magnusForce', dynamicPressure * p.area * abs(cl));
end

function cd = smoothSphereCd(reynolds)
    re = max(reynolds, 1e-3);
    first = 24 / re;
    second = (2.6 * (re / 5)) / (1 + (re / 5)^1.52);
    ratio = re / 263000;
    third = (0.411 * ratio^(-7.94)) / (1 + ratio^(-8));
    fourth = re^0.8 / 461000;
    cd = clampValue(first + second + third + fourth, 0.08, 30);
end

function prediction = interpolateTrajectory(replay, times)
    prediction = table();
    prediction.t = times;
    prediction.x = clampInterp(replay.t, replay.x, times);
    prediction.y = clampInterp(replay.t, replay.y, times);
end

function values = clampInterp(sourceTime, sourceValues, targetTime)
    values = interp1(sourceTime, sourceValues, targetTime, 'linear', 'extrap');
    values(targetTime <= sourceTime(1)) = sourceValues(1);
    values(targetTime >= sourceTime(end)) = sourceValues(end);
end

function rmse = trajectoryRmse(observed, replay)
    prediction = interpolateTrajectory(replay, observed.t);
    rmse = sqrt(mean((observed.x - prediction.x).^2 + ...
        (observed.y - prediction.y).^2));
end

function value = clampValue(value, lowerBound, upperBound)
    value = max(lowerBound, min(upperBound, value));
end

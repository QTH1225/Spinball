%% 旋转球三维轨迹：马格努斯力与空气阻力
% 这是网页模拟器的可复现实例。脚本沿用网页模型的坐标约定：
% x 为前进方向，y 为竖直方向，z 为横向方向；初始位置为原点。
% 保存为 MATLAB Live Script 后，可以在每个分节中单独查看公式、参数、结果与图形。

clear; close all; clc;

%% 1. 默认参数
p = struct();
p.ballName = "网球";
p.mass = 0.057;                 % kg
p.radius = 0.0335;              % m
p.diameter = 2 * p.radius;      % m
p.area = pi * p.radius^2;       % m^2
p.cd = 0.62;                    % 网球代表性阻力系数
p.smooth = false;               % 是否使用光滑球 Cd(Re) 关联式

p.speed = 22;                   % 初速度 m/s
p.angleDeg = 45;                % 发射角 deg
p.spinRPM = 1200;               % 自转速度 rpm
p.wind = 0;                     % x 方向平均风速 m/s
p.lateralWind = 0;              % z 方向侧向风速 m/s
p.density = 1.225;              % 海平面空气密度 kg/m^3
p.spinDecay = 0.40;             % 旋转衰减率 s^-1
p.gust = 0;                     % 阵风幅值 m/s
p.axisTiltDeg = 35;             % 旋转轴倾角 deg
p.axisAzimuthDeg = 90;          % 旋转轴方位角 deg
p.physicsMode = "advanced";    % advanced 或 basic
p.g = 9.81;                     % m/s^2
p.airViscosity = 1.81e-5;       % Pa*s
p.gasConstant = 287.05;         % J/(kg*K)
p.airTemperature = 288.15;     % K
p.dt = 0.008;                   % RK4 时间步长 s
p.maxTime = 8;                  % 最大计算时间 s
p.spinRad = p.spinRPM * 2 * pi / 60;

fprintf("三维旋转球轨迹 MATLAB 实时脚本\n");
fprintf("球类：%s | 初速度：%.1f m/s | 发射角：%.1f deg | 自转：%.0f rpm\n", ...
    p.ballName, p.speed, p.angleDeg, p.spinRPM);

%% 2. 使用的公式
% 状态向量：s = [x, y, z, vx, vy, vz]^T。
% 初始条件：r(0) = [0, 0, 0]^T，v(0) = [U cos(theta), U sin(theta), 0]^T。
%
% 相对气流速度：v_rel = v - w。
% 进阶模型的空气密度：rho(y) = rho_0 exp[-g max(y,0)/(R T)]。
% x 方向风场：w_x = w_0 + A_g sin(2 pi*0.65 t + 0.12 max(y,0)) + 0.025 w_0 max(y,0)。
% z 方向阵风：w_z = w_lateral + 0.35 A_g cos(2 pi*0.55 t)。
%
% 旋转向量：omega(t) = omega_0 exp(-lambda t) [sin(theta_s) cos(phi_s),
%             sin(theta_s) sin(phi_s), cos(theta_s)]^T。
% Reynolds 数：Re = rho |v_rel| D / mu；旋转参数：S = |omega| R / |v_rel|。
% 动压：q = 1/2 rho |v_rel|^2；受力面积：A = pi R^2；排开体积：V = 4/3 pi R^3。
% 阻力系数：普通球采用给定 Cd；光滑球使用 Morrison 型 Cd(Re) 关联式。
% 进阶升力系数：Cl = clamp[sign(S) 1.55 |S|/(1 + 1.8 |S|), -0.72, 0.72]。
% 阻力：F_D = -q Cd A v_rel/|v_rel|。
% 浮力：F_B = rho V g e_y。
% 马格努斯力：F_M = q Cl A (omega x v_rel)/|omega x v_rel|。
% 运动方程：m a = m[0,-g,0]^T + F_D + F_B + F_M。
% 数值方法：对一阶状态方程使用四阶 Runge-Kutta（RK4）积分。

formulaText = [ ...
    "状态：s = [x, y, z, vx, vy, vz]^T"; ...
    "相对速度：v_rel = v - w"; ...
    "密度：rho(y) = rho_0 exp(-g*max(y,0)/(R*T))"; ...
    "Re = rho*|v_rel|*D/mu，S = |omega|*R/|v_rel|"; ...
    "q = 0.5*rho*|v_rel|^2，A = pi*R^2，V = 4*pi*R^3/3"; ...
    "F_D = -q*Cd*A*v_rel/|v_rel|"; ...
    "F_B = rho*V*g*e_y"; ...
    "F_M = q*Cl*A*(omega x v_rel)/|omega x v_rel|"; ...
    "m*a = m*[0,-g,0]^T + F_D + F_B + F_M"; ...
    "RK4：s_(n+1) = s_n + dt*(k1 + 2*k2 + 2*k3 + k4)/6" ...
    ];
disp(formulaText);

%% 3. 三组工况计算
vacuum = simulate3D(p, "vacuum");
dragOnly = simulate3D(p, "drag");
fullModel = simulate3D(p, "full");
fullHistory = diagnosticHistory(fullModel, p);

%% 4. 结果汇总
caseNames = ["无空气"; "仅空气阻力"; "阻力 + 旋转"];
resultTable = table(caseNames, ...
    [vacuum.range; dragOnly.range; fullModel.range], ...
    [vacuum.height; dragOnly.height; fullModel.height], ...
    [vacuum.time; dragOnly.time; fullModel.time], ...
    [vacuum.sideRange; dragOnly.sideRange; fullModel.sideRange], ...
    'VariableNames', {'工况', '落点距离_m', '最高点_m', '飞行时间_s', '横向偏移_m'});
disp(resultTable);

initial = fullModel.diagnostics.initial;
fprintf("\n完整模型初始诊断：\n");
fprintf("Re = %.3g，S = %.4f，Cd = %.4f，Cl = %.4f\n", ...
    initial.reynolds, initial.spinParameter, initial.cd, initial.cl);
fprintf("阻力 = %.4f N，马格努斯力 = %.4f N，浮力 = %.5f N\n", ...
    initial.dragForce, initial.magnusForce, initial.buoyancyForce);
fprintf("旋转抬升高度（相对仅阻力工况） = %.4f m\n", ...
    fullModel.height - dragOnly.height);

%% 5. 三维轨迹与投影
figure('Color', 'w', 'Name', '三维旋转球轨迹');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
plot3(vacuum.points(:,2), vacuum.points(:,4), vacuum.points(:,3), ...
    'Color', [0.55 0.65 0.60], 'LineWidth', 1.5);
plot3(dragOnly.points(:,2), dragOnly.points(:,4), dragOnly.points(:,3), ...
    'Color', [0.95 0.35 0.20], 'LineWidth', 1.8);
plot3(fullModel.points(:,2), fullModel.points(:,4), fullModel.points(:,3), ...
    'Color', [0.10 0.35 0.95], 'LineWidth', 2.2);
scatter3(0, 0, 0, 45, [0.07 0.24 0.19], 'filled');
scatter3(fullModel.points(end,2), fullModel.points(end,4), fullModel.points(end,3), ...
    65, [0.95 0.49 0.20], 'filled');
grid on; box on; view(42, 28); axis tight;
xlabel('x / m（前进方向）');
ylabel('z / m（横向方向）');
zlabel('y / m（竖直方向）');
title('三维轨迹：图中竖直轴对应物理 y');
legend({'无空气', '仅空气阻力', '阻力 + 旋转', '起点', '落点'}, ...
    'Location', 'best');

nexttile;
hold on;
plot(fullModel.points(:,2), fullModel.points(:,3), 'Color', [0.10 0.35 0.95], 'LineWidth', 2);
plot(fullModel.points(:,2), fullModel.points(:,4), 'Color', [0.95 0.35 0.20], 'LineWidth', 1.6);
yline(0, ':', 'Color', [0.35 0.40 0.38]);
grid on; box on;
xlabel('x / m');
ylabel('位移 / m');
title('纵向高度与横向偏移投影');
legend({'竖直高度 y', '横向偏移 z', '地面'}, 'Location', 'best');

%% 6. 气动诊断量随时间变化
figure('Color', 'w', 'Name', '气动诊断量');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(fullHistory.t, fullHistory.reynolds, 'Color', [0.07 0.45 0.32], 'LineWidth', 1.8);
grid on; box on;
xlabel('t / s'); ylabel('Re'); title('Reynolds 数');

nexttile;
plot(fullHistory.t, fullHistory.spinParameter, 'Color', [0.45 0.25 0.78], 'LineWidth', 1.8);
grid on; box on;
xlabel('t / s'); ylabel('S'); title('旋转参数 S');

nexttile;
plot(fullHistory.t, fullHistory.dragForce, 'Color', [0.95 0.35 0.20], 'LineWidth', 1.8);
hold on;
plot(fullHistory.t, fullHistory.magnusForce, 'Color', [0.10 0.35 0.95], 'LineWidth', 1.8);
grid on; box on;
xlabel('t / s'); ylabel('力 / N'); title('阻力与马格努斯力');
legend({'|F_D|', '|F_M|'}, 'Location', 'best');

nexttile;
plot(fullHistory.t, fullHistory.cl, 'Color', [0.07 0.24 0.19], 'LineWidth', 1.8);
hold on;
plot(fullHistory.t, fullHistory.cd, 'Color', [0.95 0.49 0.20], 'LineWidth', 1.8);
grid on; box on;
xlabel('t / s'); ylabel('系数'); title('升力系数与阻力系数');
legend({'Cl', 'Cd'}, 'Location', 'best');

%% 7. RK4 步长收敛检查
fine = simulate3D(setfield(p, 'dt', p.dt / 2), "full"); %#ok<SFLD>
stepDifference = abs(fullModel.range - fine.range);
heightDifference = abs(fullModel.height - fine.height);
fprintf("\nRK4 步长检查：dt = %.4f s，dt/2 = %.4f s\n", p.dt, p.dt / 2);
fprintf("落点距离差 = %.6g m，最高点差 = %.6g m\n", stepDifference, heightDifference);

%% 8. 说明
% 触地判定：当一步积分后 y < 0 时，在上一点与当前点之间线性插值，令 y = 0，
% 从而得到比直接取离散点更稳定的落点时间与落点位置。
% 对普通球，Cd 使用球类预设值；只有 p.smooth = true 时才启用 Cd(Re) 关联式。
% 该脚本与网页模型保持一致，适合复核轨迹、诊断量和控制变量结果；不代替真实风洞或实测标定。

%% 局部函数
function result = simulate3D(p, mode)
    angle = deg2rad(p.angleDeg);
    state = [0; 0; 0; p.speed*cos(angle); p.speed*sin(angle); 0];
    t = 0;
    points = [t, state.'];
    initialAero = aerodynamicState3D(state, p, 0);
    nSteps = floor(p.maxTime / p.dt);

    for step = 1:nSteps
        previousState = state;
        state = rk4Step3D(state, p, p.dt, mode, t);
        t = t + p.dt;
        if state(2) < 0 && t > p.dt
            fraction = previousState(2) / max(previousState(2) - state(2), 1e-12);
            landingState = previousState + fraction * (state - previousState);
            landingState(2) = 0;
            landingTime = t - p.dt + p.dt * fraction;
            points(end+1,:) = [landingTime, landingState.'];
            break;
        end
        state(2) = max(0, state(2));
        points(end+1,:) = [t, state.'];
    end

    lastState = points(end, 2:7).';
    finalAero = aerodynamicState3D(lastState, p, points(end,1));
    result.points = points;
    result.range = hypot(lastState(1), lastState(3));
    result.sideRange = lastState(3);
    result.height = max(points(:,3));
    result.time = points(end,1);
    result.mode = mode;
    result.dt = p.dt;
    result.diagnostics.initial = initialAero;
    result.diagnostics.final = finalAero;
    result.diagnostics.spinEnd = finalAero.spinRad * 60 / (2*pi);
end

function nextState = rk4Step3D(state, p, dt, mode, t)
    k1 = derivative3D(state, p, mode, t);
    k2 = derivative3D(state + (dt/2)*k1, p, mode, t + dt/2);
    k3 = derivative3D(state + (dt/2)*k2, p, mode, t + dt/2);
    k4 = derivative3D(state + dt*k3, p, mode, t + dt);
    nextState = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function derivative = derivative3D(state, p, mode, t)
    acceleration = [0; -p.g; 0];
    if ~strcmp(mode, "vacuum")
        aero = aerodynamicState3D(state, p, t);
        if aero.relativeSpeed > 1e-8
            dragAcceleration = -0.5 * aero.density * aero.cd * p.area * aero.relativeSpeed ...
                * aero.relativeVelocity / p.mass;
            acceleration = acceleration + dragAcceleration;
            acceleration(2) = acceleration(2) + aero.buoyancyForce / p.mass;

            if strcmp(mode, "full") && abs(aero.spinParameter) > 1e-8
                crossVector = cross(aero.spinVector, aero.relativeVelocity);
                crossMagnitude = norm(crossVector);
                if crossMagnitude > 1e-8
                    magnusFactor = 0.5 * aero.density * p.area * aero.cl ...
                        * aero.relativeSpeed^2 / p.mass;
                    acceleration = acceleration + magnusFactor * crossVector / crossMagnitude;
                end
            end
        end
    end
    derivative = [state(4:6); acceleration];
end

function aero = aerodynamicState3D(state, p, t)
    y = state(2);
    if strcmp(p.physicsMode, "advanced")
        density = p.density * exp(-p.g * max(y,0) / (p.gasConstant * p.airTemperature));
    else
        density = p.density;
    end
    wind = windVectorAt(y, t, p);
    velocity = state(4:6);
    relativeVelocity = velocity - wind;
    relativeSpeed = norm(relativeVelocity);
    spinVector = spinVectorAt(p, t);
    spinRad = norm(spinVector);
    spinParameter = 0;
    if relativeSpeed > 1e-5
        spinParameter = spinRad * p.radius / relativeSpeed;
    end
    reynolds = density * relativeSpeed * p.diameter / p.airViscosity;
    cd = p.cd;
    cl = clampValue(0.000115 * (spinRad * 60 / (2*pi)), -0.34, 0.34);
    if strcmp(p.physicsMode, "advanced")
        if p.smooth
            cd = smoothSphereCd(reynolds);
        else
            cd = p.cd;
        end
        cl = clampValue(sign(spinParameter) * (1.55 * abs(spinParameter)) ...
            / (1 + 1.8 * abs(spinParameter)), -0.72, 0.72);
    end
    dynamicPressure = 0.5 * density * relativeSpeed^2;
    displacedVolume = (4/3) * pi * p.radius^3;
    buoyancyForce = density * displacedVolume * p.g;
    aero.density = density;
    aero.wind = wind;
    aero.relativeVelocity = relativeVelocity;
    aero.relativeSpeed = relativeSpeed;
    aero.spinVector = spinVector;
    aero.spinRad = spinRad;
    aero.spinParameter = spinParameter;
    aero.reynolds = reynolds;
    aero.cd = cd;
    aero.cl = cl;
    aero.dynamicPressure = dynamicPressure;
    aero.buoyancyForce = buoyancyForce;
    aero.dragForce = dynamicPressure * p.area * cd;
    aero.magnusForce = dynamicPressure * p.area * abs(cl);
end

function wind = windVectorAt(y, t, p)
    if strcmp(p.physicsMode, "advanced")
        gustX = p.gust * sin(2*pi*0.65*t + 0.12*max(y,0));
        shear = 0.025 * p.wind * max(y,0);
        lateralGust = 0.35 * p.gust * cos(2*pi*0.55*t);
    else
        gustX = 0;
        shear = 0;
        lateralGust = 0;
    end
    wind = [p.wind + gustX + shear; 0; p.lateralWind + lateralGust];
end

function spinVector = spinVectorAt(p, t)
    spinRad = p.spinRad;
    if strcmp(p.physicsMode, "advanced")
        spinRad = spinRad * exp(-p.spinDecay * t);
    end
    tilt = deg2rad(p.axisTiltDeg);
    azimuth = deg2rad(p.axisAzimuthDeg);
    spinVector = spinRad * [sin(tilt)*cos(azimuth); sin(tilt)*sin(azimuth); cos(tilt)];
end

function cd = smoothSphereCd(reynolds)
    re = max(reynolds, 1e-3);
    first = 24 / re;
    second = (2.6 * (re/5)) / (1 + (re/5)^1.52);
    ratio = re / 263000;
    third = (0.411 * ratio^(-7.94)) / (1 + ratio^(-8));
    fourth = re^0.8 / 461000;
    cd = clampValue(first + second + third + fourth, 0.08, 30);
end

function history = diagnosticHistory(result, p)
    n = size(result.points, 1);
    history.t = result.points(:,1);
    history.reynolds = zeros(n,1);
    history.spinParameter = zeros(n,1);
    history.cd = zeros(n,1);
    history.cl = zeros(n,1);
    history.dragForce = zeros(n,1);
    history.magnusForce = zeros(n,1);
    for i = 1:n
        state = result.points(i,2:7).';
        aero = aerodynamicState3D(state, p, result.points(i,1));
        history.reynolds(i) = aero.reynolds;
        history.spinParameter(i) = aero.spinParameter;
        history.cd(i) = aero.cd;
        history.cl(i) = aero.cl;
        history.dragForce(i) = aero.dragForce;
        history.magnusForce(i) = aero.magnusForce;
    end
end

function value = clampValue(value, lowerBound, upperBound)
    value = min(max(value, lowerBound), upperBound);
end

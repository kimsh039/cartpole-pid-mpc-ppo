clear;
clc;
close all;

paths = project_paths();

%% Load trajectories generated under the same conditions

pid = readtable(fullfile(paths.Trajectories,"pid_result.csv"));
mpc = readtable(fullfile(paths.Trajectories,"mpc_result.csv"));
ppo = readtable(fullfile(paths.Trajectories,"ppo_result.csv"));

names = ["PID"; "MPC"; "PPO"];
data = {pid; mpc; ppo};

success = false(3,1);
angleRMSEDeg = zeros(3,1);
positionRMSE = zeros(3,1);
maximumAngleDeg = zeros(3,1);
maximumPosition = zeros(3,1);
controlEnergy = zeros(3,1);

for index = 1:3
    result = data{index};
    dt = result.time(2) - result.time(1);

    success(index) = ...
        result.time(end) >= 10.0 - dt ...
        && max(abs(rad2deg(result.angle))) <= 12.0 ...
        && max(abs(result.position)) <= 2.4;

    angleRMSEDeg(index) = ...
        sqrt(mean(rad2deg(result.angle).^2));

    positionRMSE(index) = ...
        sqrt(mean(result.position.^2));

    maximumAngleDeg(index) = ...
        max(abs(rad2deg(result.angle)));

    maximumPosition(index) = ...
        max(abs(result.position));

    controlEnergy(index) = ...
        sum(result.force(1:end - 1).^2)*dt;
end

comparisonTable = table( ...
    names, ...
    success, ...
    angleRMSEDeg, ...
    positionRMSE, ...
    maximumAngleDeg, ...
    maximumPosition, ...
    controlEnergy, ...
    VariableNames=[ ...
        "Controller", ...
        "Success", ...
        "AngleRMSEDeg", ...
        "PositionRMSEm", ...
        "MaxAngleDeg", ...
        "MaxPositionm", ...
        "ControlEnergyN2s" ...
    ]);

disp(comparisonTable);
writetable( ...
    comparisonTable, ...
    fullfile(paths.Tables,"controller_comparison.csv"));

%% Overlay plots

figure("Name","PID MPC PPO Comparison", ...
    "Position",[100 100 1200 850]);
tiledlayout(3,1);

nexttile;
plot(pid.time,rad2deg(pid.angle),"LineWidth",1.4);
hold on;
plot(mpc.time,rad2deg(mpc.angle),"LineWidth",1.4);
plot(ppo.time,rad2deg(ppo.angle),"LineWidth",1.4);
yline(12,"k:");
yline(-12,"k:");
grid on;
ylabel("Angle [deg]");
title("Pole Angle");
legend("PID","MPC","PPO","Location","best");

nexttile;
plot(pid.time,pid.position,"LineWidth",1.4);
hold on;
plot(mpc.time,mpc.position,"LineWidth",1.4);
plot(ppo.time,ppo.position,"LineWidth",1.4);
grid on;
ylabel("Position [m]");
title("Cart Position");

nexttile;
plot(pid.time,pid.force,"LineWidth",1.2);
hold on;
plot(mpc.time,mpc.force,"LineWidth",1.2);
plot(ppo.time,ppo.force,"LineWidth",1.2);
grid on;
xlabel("Time [s]");
ylabel("Force [N]");
title("Control Input");

sgtitle("CartPole Controller Comparison: Same Nonlinear Plant");

outputPath = fullfile( ...
    paths.Images, ...
    "controller_response_comparison.png");

exportgraphics(gcf,outputPath,"Resolution",180);

fprintf("Saved: %s\n", ...
    fullfile(paths.Tables,"controller_comparison.csv"));
fprintf("Saved: %s\n",outputPath);

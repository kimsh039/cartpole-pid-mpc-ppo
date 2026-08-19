clear;
clc;
close all;

paths = project_paths();

%% 1. Load the trained PPO agent and parameters

load( ...
    fullfile(paths.Models,"ppo_cartpole_trained.mat"), ...
    "agent");

agent.UseExplorationPolicy = false;

p = cartpole_params();
dt = p.dt;
duration = 10.0;
numberOfSteps = round(duration/dt);

%% 2. Use exactly the same initial condition as PID and MPC

initialState = [
    0.20
    0.00
    deg2rad(5)
    0.00
];

%% 3. Allocate histories

time = (0:numberOfSteps)'*dt;
stateHistory = zeros(numberOfSteps + 1,4);
forceHistory = zeros(numberOfSteps + 1,1);
calculationTimeHistory = zeros(numberOfSteps + 1,1);

stateHistory(1,:) = initialState';

lastIndex = numberOfSteps + 1;
failureReason = "none";

% Warm up neural-network inference so initialization time is not measured.
warmupObservation = normalize_cartpole_observation(initialState,p);
for warmupIndex = 1:10
    getAction(agent,{warmupObservation});
end

%% 4. Closed-loop PPO control on the nonlinear plant

for step = 1:numberOfSteps

    currentState = stateHistory(step,:)';

    observation = ...
        normalize_cartpole_observation(currentState,p);

    timerStart = tic;
    actionOutput = getAction(agent,{observation});
    calculationTimeHistory(step) = toc(timerStart);

    if iscell(actionOutput)
        appliedForce = double(actionOutput{1}(1));
    else
        appliedForce = double(actionOutput(1));
    end

    appliedForce = min( ...
        max(appliedForce,-p.force_limit), ...
        p.force_limit);

    nextState = rk4_step( ...
        currentState, ...
        appliedForce, ...
        dt, ...
        p);

    stateHistory(step + 1,:) = nextState';
    forceHistory(step) = appliedForce;

    if abs(nextState(3)) > p.angle_limit
        lastIndex = step + 1;
        failureReason = "pole angle limit";
        break;
    end

    if abs(nextState(1)) > p.position_limit
        lastIndex = step + 1;
        failureReason = "cart position limit";
        break;
    end
end

%% 5. Trim results

time = time(1:lastIndex);
stateHistory = stateHistory(1:lastIndex,:);
forceHistory = forceHistory(1:lastIndex);
calculationTimeHistory = calculationTimeHistory(1:lastIndex);

if lastIndex > 1
    forceHistory(end) = forceHistory(end - 1);
    calculationTimeHistory(end) = calculationTimeHistory(end - 1);
end

position = stateHistory(:,1);
positionVelocity = stateHistory(:,2);
angle = stateHistory(:,3);
angleVelocity = stateHistory(:,4);

%% 6. Calculate metrics without double-counting the final plotted sample

controlSamples = 1:max(lastIndex - 1,1);

angleRMSEDeg = sqrt(mean(rad2deg(angle).^2));
positionRMSE = sqrt(mean(position.^2));
maximumAngleDeg = max(abs(rad2deg(angle)));
maximumPosition = max(abs(position));
controlEnergy = sum(forceHistory(controlSamples).^2)*dt;
averageCalculationTimeMs = ...
    mean(calculationTimeHistory(controlSamples))*1000;
maximumCalculationTimeMs = ...
    max(calculationTimeHistory(controlSamples))*1000;
success = lastIndex == numberOfSteps + 1;

fprintf("\n=== PPO RESULT ===\n");
fprintf("Success: %d\n",success);
fprintf("Failure reason: %s\n",failureReason);
fprintf("Simulation time: %.2f s\n",time(end));
fprintf("Angle RMSE: %.4f deg\n",angleRMSEDeg);
fprintf("Position RMSE: %.4f m\n",positionRMSE);
fprintf("Maximum angle: %.4f deg\n",maximumAngleDeg);
fprintf("Maximum position: %.4f m\n",maximumPosition);
fprintf("Control effort: %.4f N^2*s\n",controlEnergy);
fprintf("Average inference: %.4f ms\n",averageCalculationTimeMs);
fprintf("Maximum inference: %.4f ms\n",maximumCalculationTimeMs);

%% 7. Plot result

figure("Name","PPO on Nonlinear CartPole");
tiledlayout(3,1);

nexttile;
plot(time,rad2deg(angle),"LineWidth",1.6);
hold on;
yline(rad2deg(p.angle_limit),"r--");
yline(-rad2deg(p.angle_limit),"r--");
grid on;
ylabel("Angle [deg]");
title("Pole Angle");

nexttile;
plot(time,position,"LineWidth",1.6);
grid on;
ylabel("Position [m]");
title("Cart Position");

nexttile;
plot(time,forceHistory,"LineWidth",1.4);
hold on;
yline(p.force_limit,"r--");
yline(-p.force_limit,"r--");
grid on;
xlabel("Time [s]");
ylabel("Force [N]");
title("PPO Control Force");

sgtitle("Trained PPO Controlling Nonlinear CartPole");

ppoFigurePath = fullfile( ...
    paths.Images, ...
    "ppo_control_result.png");

exportgraphics(gcf,ppoFigurePath,"Resolution",180);

%% 8. Save trajectory for comparison and 3-D visualization

resultTable = table( ...
    time, ...
    position, ...
    positionVelocity, ...
    angle, ...
    angleVelocity, ...
    forceHistory, ...
    calculationTimeHistory, ...
    VariableNames=[ ...
        "time", ...
        "position", ...
        "positionVelocity", ...
        "angle", ...
        "angleVelocity", ...
        "force", ...
        "calculationTime" ...
    ]);

writetable( ...
    resultTable, ...
    fullfile(paths.Trajectories,"ppo_result.csv"));
fprintf("Saved: %s\n", ...
    fullfile(paths.Trajectories,"ppo_result.csv"));
fprintf("Saved: %s\n",ppoFigurePath);

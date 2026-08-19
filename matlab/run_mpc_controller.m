clear;
clc;
close all;

paths = project_paths();

mpcverbosity("off");

%% 1. Load model and parameters

load(fullfile(paths.Models,"cartpole_linear_model.mat"));

dt = p.dt;
duration = 10.0;
numberOfSteps = round(duration/dt);

%% 2. Construct discrete linear prediction model

linearPlant = ss( ...
    Ad, ...
    Bd, ...
    eye(4), ...
    zeros(4,1), ...
    dt);

%% 3. MPC horizons

predictionHorizon = 40;
controlHorizon = 10;

mpcController = mpc( ...
    linearPlant, ...
    dt, ...
    predictionHorizon, ...
    controlHorizon);

%% 4. MPC cost weights

mpcController.Weights.OutputVariables = [
    20.0 ...    % Cart position
    1.0  ...    % Cart velocity
    100.0 ...   % Pole angle
    2.0         % Pole angular velocity
];

mpcController.Weights.ManipulatedVariables = 0.05;

mpcController.Weights.ManipulatedVariablesRate = 0.05;

%% 5. Input constraints

mpcController.MV.Min = -p.force_limit;
mpcController.MV.Max =  p.force_limit;

%% 6. Reference state

referenceState = [
    0
    0
    0
    0
];

%% 7. Initial state

initialState = [
    0.20
    0.00
    deg2rad(5)
    0.00
];

%% 8. MPC internal state

controllerState = mpcstate(mpcController);

controllerState.Plant = initialState;

%% 9. Allocate result arrays

time = (0:numberOfSteps)'*dt;

stateHistory = zeros(numberOfSteps + 1,4);
forceHistory = zeros(numberOfSteps + 1,1);
calculationTimeHistory = zeros(numberOfSteps + 1,1);
predictedCostHistory = zeros(numberOfSteps + 1,1);
iterationHistory = zeros(numberOfSteps + 1,1);

stateHistory(1,:) = initialState';

lastIndex = numberOfSteps + 1;
failureReason = "none";

%% 10. Receding-horizon control loop

for step = 1:numberOfSteps

    currentState = stateHistory(step,:)';

    % We assume all four states are directly measured.
    controllerState.Plant = currentState;

    %% Solve the MPC optimization problem

    timerStart = tic;

    [appliedForce,moveInfo] = mpcmove( ...
        mpcController, ...
        controllerState, ...
        currentState, ...
        referenceState);

    calculationTime = toc(timerStart);

    %% Apply the force to the nonlinear plant

    nextState = rk4_step( ...
        currentState, ...
        appliedForce, ...
        dt, ...
        p);

    %% Save data

    stateHistory(step + 1,:) = nextState';
    forceHistory(step) = appliedForce;
    calculationTimeHistory(step) = calculationTime;
    predictedCostHistory(step) = moveInfo.Cost;
    iterationHistory(step) = moveInfo.Iterations;

    %% Failure conditions

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

%% 11. Trim unused values

time = time(1:lastIndex);
stateHistory = stateHistory(1:lastIndex,:);
forceHistory = forceHistory(1:lastIndex);
calculationTimeHistory = ...
    calculationTimeHistory(1:lastIndex);
predictedCostHistory = ...
    predictedCostHistory(1:lastIndex);
iterationHistory = ...
    iterationHistory(1:lastIndex);

if lastIndex > 1
    forceHistory(end) = forceHistory(end - 1);

    calculationTimeHistory(end) = ...
        calculationTimeHistory(end - 1);

    predictedCostHistory(end) = ...
        predictedCostHistory(end - 1);

    iterationHistory(end) = ...
        iterationHistory(end - 1);
end

%% 12. Extract state variables

position = stateHistory(:,1);
positionVelocity = stateHistory(:,2);
angle = stateHistory(:,3);
angleVelocity = stateHistory(:,4);

%% 13. Calculate performance metrics

angleRMSEDeg = sqrt( ...
    mean(rad2deg(angle).^2));

positionRMSE = sqrt( ...
    mean(position.^2));

maximumAngleDeg = max( ...
    abs(rad2deg(angle)));

maximumPosition = max( ...
    abs(position));

controlEnergy = sum( ...
    forceHistory.^2)*dt;

averageCalculationTimeMs = ...
    mean(calculationTimeHistory(1:end - 1))*1000;

maximumCalculationTimeMs = ...
    max(calculationTimeHistory(1:end - 1))*1000;

averageSolverIterations = ...
    mean(iterationHistory(1:end - 1));

success = ...
    lastIndex == numberOfSteps + 1;

fprintf("\n=== MPC RESULT ===\n");
fprintf("Success: %d\n",success);
fprintf("Failure reason: %s\n",failureReason);
fprintf("Simulation time: %.2f s\n",time(end));
fprintf("Angle RMSE: %.4f deg\n",angleRMSEDeg);
fprintf("Position RMSE: %.4f m\n",positionRMSE);
fprintf("Maximum angle: %.4f deg\n",maximumAngleDeg);
fprintf("Maximum position: %.4f m\n",maximumPosition);
fprintf("Control effort: %.4f N^2*s\n",controlEnergy);
fprintf("Average computation: %.4f ms\n", ...
    averageCalculationTimeMs);
fprintf("Maximum computation: %.4f ms\n", ...
    maximumCalculationTimeMs);
fprintf("Average solver iterations: %.2f\n", ...
    averageSolverIterations);

%% 14. Plot results

figure("Name","Linear MPC on Nonlinear CartPole");

tiledlayout(4,1);

nexttile;

plot( ...
    time, ...
    rad2deg(angle), ...
    "b-", ...
    "LineWidth",1.6);

hold on;

yline(0,"r--");

yline( ...
    rad2deg(p.angle_limit), ...
    "k:", ...
    "Failure limit");

yline( ...
    -rad2deg(p.angle_limit), ...
    "k:");

grid on;
ylabel("Angle [deg]");
title("Pole Angle");

nexttile;

plot( ...
    time, ...
    position, ...
    "LineWidth",1.6);

hold on;
yline(0,"r--");

grid on;
ylabel("Position [m]");
title("Cart Position");

nexttile;

plot( ...
    time, ...
    forceHistory, ...
    "LineWidth",1.6);

hold on;
yline(p.force_limit,"r--");
yline(-p.force_limit,"r--");

grid on;
ylabel("Force [N]");
title("MPC Control Force");

nexttile;

plot( ...
    time, ...
    calculationTimeHistory*1000, ...
    "LineWidth",1.3);

hold on;
yline(dt*1000,"r--","20 ms deadline");

grid on;
xlabel("Time [s]");
ylabel("Time [ms]");
title("MPC Computation Time");

sgtitle("Linear MPC Controlling Nonlinear CartPole");

%% 15. Save results

resultTable = table( ...
    time, ...
    position, ...
    positionVelocity, ...
    angle, ...
    angleVelocity, ...
    forceHistory, ...
    calculationTimeHistory, ...
    predictedCostHistory, ...
    iterationHistory, ...
    VariableNames=[ ...
        "time", ...
        "position", ...
        "positionVelocity", ...
        "angle", ...
        "angleVelocity", ...
        "force", ...
        "calculationTime", ...
        "predictedCost", ...
        "solverIterations" ...
    ]);

writetable( ...
    resultTable, ...
    fullfile(paths.Trajectories,"mpc_result.csv"));

fprintf("Saved: %s\n", ...
    fullfile(paths.Trajectories,"mpc_result.csv"));

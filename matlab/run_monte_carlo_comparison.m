clear;
clc;
close all;

paths = project_paths();

mpcverbosity("off");
rng(2026,"twister");

%% 1. Shared experiment conditions

p = cartpole_params();
duration = 10.0;
numberOfSteps = round(duration/p.dt);
numberOfTrials = 30;

initialStates = zeros(4,numberOfTrials);
initialStates(1,:) = -0.5 + rand(1,numberOfTrials);
initialStates(2,:) = -0.2 + 0.4*rand(1,numberOfTrials);
initialStates(3,:) = deg2rad(-8 + 16*rand(1,numberOfTrials));
initialStates(4,:) = -0.3 + 0.6*rand(1,numberOfTrials);

%% 2. Construct the MPC controller once

load( ...
    fullfile(paths.Models,"cartpole_linear_model.mat"), ...
    "Ad","Bd");

linearPlant = ss( ...
    Ad, ...
    Bd, ...
    eye(4), ...
    zeros(4,1), ...
    p.dt);

mpcController = mpc(linearPlant,p.dt,40,10);
mpcController.Weights.OutputVariables = [20 1 100 2];
mpcController.Weights.ManipulatedVariables = 0.05;
mpcController.Weights.ManipulatedVariablesRate = 0.05;
mpcController.MV.Min = -p.force_limit;
mpcController.MV.Max = p.force_limit;

%% 3. Load the deterministic trained PPO policy

load( ...
    fullfile(paths.Models,"ppo_cartpole_trained.mat"), ...
    "agent");
agent.UseExplorationPolicy = false;

warmupObservation = ...
    normalize_cartpole_observation(initialStates(:,1),p);

for warmupIndex = 1:10
    getAction(agent,{warmupObservation});
end

%% 4. Run every controller on exactly the same initial states

controllerNames = ["PID"; "MPC"; "PPO"];
numberOfControllers = numel(controllerNames);

success = false(numberOfControllers,numberOfTrials);
survivalTime = zeros(numberOfControllers,numberOfTrials);
angleRMSEDeg = zeros(numberOfControllers,numberOfTrials);
positionRMSE = zeros(numberOfControllers,numberOfTrials);
controlEnergy = zeros(numberOfControllers,numberOfTrials);

for controllerIndex = 1:numberOfControllers

    controllerName = controllerNames(controllerIndex);
    fprintf("Running %s: ",controllerName);

    for trialIndex = 1:numberOfTrials

        initialState = initialStates(:,trialIndex);

        switch controllerName
            case "PID"
                result = simulate_pid( ...
                    initialState,p,numberOfSteps);

            case "MPC"
                result = simulate_mpc( ...
                    initialState,p,numberOfSteps,mpcController);

            case "PPO"
                result = simulate_ppo( ...
                    initialState,p,numberOfSteps,agent);
        end

        success(controllerIndex,trialIndex) = result.Success;
        survivalTime(controllerIndex,trialIndex) = result.SurvivalTime;
        angleRMSEDeg(controllerIndex,trialIndex) = result.AngleRMSEDeg;
        positionRMSE(controllerIndex,trialIndex) = result.PositionRMSE;
        controlEnergy(controllerIndex,trialIndex) = result.ControlEnergy;

        fprintf(".");
    end

    fprintf(" done\n");
end

%% 5. Per-trial output

controllerColumn = repelem(controllerNames,numberOfTrials);
trialColumn = repmat((1:numberOfTrials)',numberOfControllers,1);
initialStateColumn = repmat(initialStates',numberOfControllers,1);

trialTable = table( ...
    controllerColumn, ...
    trialColumn, ...
    initialStateColumn(:,1), ...
    initialStateColumn(:,2), ...
    rad2deg(initialStateColumn(:,3)), ...
    initialStateColumn(:,4), ...
    reshape(success',[],1), ...
    reshape(survivalTime',[],1), ...
    reshape(angleRMSEDeg',[],1), ...
    reshape(positionRMSE',[],1), ...
    reshape(controlEnergy',[],1), ...
    VariableNames=[ ...
        "Controller", ...
        "Trial", ...
        "InitialPositionm", ...
        "InitialVelocitymps", ...
        "InitialAngleDeg", ...
        "InitialAngularRateRadps", ...
        "Success", ...
        "SurvivalTimeS", ...
        "AngleRMSEDeg", ...
        "PositionRMSEm", ...
        "ControlEnergyN2s" ...
    ]);

writetable( ...
    trialTable, ...
    fullfile(paths.Tables,"monte_carlo_trials.csv"));

%% 6. Summary statistics

successRatePct = 100*mean(success,2);
meanSurvivalTimeS = mean(survivalTime,2);
meanAngleRMSESuccessfulDeg = nan(numberOfControllers,1);
meanPositionRMSESuccessfulm = nan(numberOfControllers,1);
meanControlEnergySuccessfulN2s = nan(numberOfControllers,1);

for controllerIndex = 1:numberOfControllers
    successfulTrials = success(controllerIndex,:);

    if any(successfulTrials)
        meanAngleRMSESuccessfulDeg(controllerIndex) = ...
            mean(angleRMSEDeg(controllerIndex,successfulTrials));

        meanPositionRMSESuccessfulm(controllerIndex) = ...
            mean(positionRMSE(controllerIndex,successfulTrials));

        meanControlEnergySuccessfulN2s(controllerIndex) = ...
            mean(controlEnergy(controllerIndex,successfulTrials));
    end
end

summaryTable = table( ...
    controllerNames, ...
    numberOfTrials*ones(numberOfControllers,1), ...
    successRatePct, ...
    meanSurvivalTimeS, ...
    meanAngleRMSESuccessfulDeg, ...
    meanPositionRMSESuccessfulm, ...
    meanControlEnergySuccessfulN2s, ...
    VariableNames=[ ...
        "Controller", ...
        "Trials", ...
        "SuccessRatePct", ...
        "MeanSurvivalTimeS", ...
        "MeanAngleRMSESuccessfulDeg", ...
        "MeanPositionRMSESuccessfulm", ...
        "MeanControlEnergySuccessfulN2s" ...
    ]);

disp(summaryTable);
writetable( ...
    summaryTable, ...
    fullfile(paths.Tables,"monte_carlo_summary.csv"));

%% 7. Presentation-ready plot

figure( ...
    "Name","Monte Carlo Controller Comparison", ...
    "Position",[100 100 1400 500], ...
    "Color","white");

tiledlayout(1,3);
controllerColors = [
    0.10 0.55 0.95
    1.00 0.45 0.05
    0.95 0.80 0.10
];

nexttile;
barHandle = bar(successRatePct,"FaceColor","flat");
barHandle.CData = controllerColors;
ylim([0 105]);
xticks(1:3);
xticklabels(controllerNames);
ylabel("Success rate [%]");
title(sprintf("Success Rate (%d Trials)",numberOfTrials));
grid on;
style_axes(gca);

nexttile;
barHandle = bar(meanAngleRMSESuccessfulDeg,"FaceColor","flat");
barHandle.CData = controllerColors;
xticks(1:3);
xticklabels(controllerNames);
ylabel("Angle RMSE [deg]");
title("Precision on Successful Trials");
grid on;
style_axes(gca);

nexttile;
barHandle = bar(meanControlEnergySuccessfulN2s,"FaceColor","flat");
barHandle.CData = controllerColors;
xticks(1:3);
xticklabels(controllerNames);
ylabel("Control energy [N^2 s]");
title("Energy on Successful Trials");
grid on;
style_axes(gca);

sgtitle( ...
    "PID vs MPC vs PPO: Shared Random Initial Conditions", ...
    "Color","black");

outputPath = fullfile( ...
    paths.Images, ...
    "monte_carlo_controller_comparison.png");

exportgraphics(gcf,outputPath,"Resolution",180);

fprintf("Saved: %s\n", ...
    fullfile(paths.Tables,"monte_carlo_trials.csv"));
fprintf("Saved: %s\n", ...
    fullfile(paths.Tables,"monte_carlo_summary.csv"));
fprintf("Saved: %s\n",outputPath);

%% Local simulation functions

function result = simulate_pid(initialState,p,numberOfSteps)

    state = initialState;
    positionIntegral = 0;
    angleIntegral = 0;
    angleSquareSum = state(3)^2;
    positionSquareSum = state(1)^2;
    energy = 0;
    stepReached = 0;
    failed = false;

    for step = 1:numberOfSteps
        positionError = state(1);
        positionIntegral = min(max( ...
            positionIntegral + positionError*p.dt,-1.0),1.0);

        angleReference = -( ...
            0.12*positionError ...
            + 0.02*positionIntegral ...
            + 0.16*state(2));

        angleReference = min(max( ...
            angleReference,-deg2rad(8)),deg2rad(8));

        angleError = state(3) - angleReference;
        angleIntegral = min(max( ...
            angleIntegral + angleError*p.dt,-0.30),0.30);

        force = ...
            60.0*angleError ...
            + 1.0*angleIntegral ...
            + 15.0*state(4);

        force = min(max(force,-p.force_limit),p.force_limit);
        state = rk4_step(state,force,p.dt,p);
        energy = energy + force^2*p.dt;
        angleSquareSum = angleSquareSum + state(3)^2;
        positionSquareSum = positionSquareSum + state(1)^2;
        stepReached = step;

        if is_failed(state,p)
            failed = true;
            break;
        end
    end

    sampleCount = stepReached + 1;
    result = make_result( ...
        ~failed && stepReached == numberOfSteps, ...
        stepReached*p.dt, ...
        angleSquareSum, ...
        positionSquareSum, ...
        sampleCount, ...
        energy);
end

function result = simulate_mpc( ...
    initialState,p,numberOfSteps,mpcController)

    state = initialState;
    controllerState = mpcstate(mpcController);
    controllerState.Plant = initialState;
    referenceState = zeros(4,1);
    angleSquareSum = state(3)^2;
    positionSquareSum = state(1)^2;
    energy = 0;
    stepReached = 0;
    failed = false;

    for step = 1:numberOfSteps
        controllerState.Plant = state;
        force = mpcmove( ...
            mpcController, ...
            controllerState, ...
            state, ...
            referenceState);

        force = min(max(force,-p.force_limit),p.force_limit);
        state = rk4_step(state,force,p.dt,p);
        energy = energy + force^2*p.dt;
        angleSquareSum = angleSquareSum + state(3)^2;
        positionSquareSum = positionSquareSum + state(1)^2;
        stepReached = step;

        if is_failed(state,p)
            failed = true;
            break;
        end
    end

    sampleCount = stepReached + 1;
    result = make_result( ...
        ~failed && stepReached == numberOfSteps, ...
        stepReached*p.dt, ...
        angleSquareSum, ...
        positionSquareSum, ...
        sampleCount, ...
        energy);
end

function result = simulate_ppo(initialState,p,numberOfSteps,agent)

    state = initialState;
    angleSquareSum = state(3)^2;
    positionSquareSum = state(1)^2;
    energy = 0;
    stepReached = 0;
    failed = false;

    for step = 1:numberOfSteps
        observation = normalize_cartpole_observation(state,p);
        actionOutput = getAction(agent,{observation});

        if iscell(actionOutput)
            force = double(actionOutput{1}(1));
        else
            force = double(actionOutput(1));
        end

        force = min(max(force,-p.force_limit),p.force_limit);
        state = rk4_step(state,force,p.dt,p);
        energy = energy + force^2*p.dt;
        angleSquareSum = angleSquareSum + state(3)^2;
        positionSquareSum = positionSquareSum + state(1)^2;
        stepReached = step;

        if is_failed(state,p)
            failed = true;
            break;
        end
    end

    sampleCount = stepReached + 1;
    result = make_result( ...
        ~failed && stepReached == numberOfSteps, ...
        stepReached*p.dt, ...
        angleSquareSum, ...
        positionSquareSum, ...
        sampleCount, ...
        energy);
end

function failed = is_failed(state,p)
    failed = ...
        abs(state(3)) > p.angle_limit ...
        || abs(state(1)) > p.position_limit ...
        || any(~isfinite(state));
end

function result = make_result( ...
    success,survivalTime,angleSquareSum,positionSquareSum,sampleCount,energy)

    result.Success = success;
    result.SurvivalTime = survivalTime;
    result.AngleRMSEDeg = ...
        rad2deg(sqrt(angleSquareSum/sampleCount));
    result.PositionRMSE = ...
        sqrt(positionSquareSum/sampleCount);
    result.ControlEnergy = energy;
end

function style_axes(axisHandle)
    axisHandle.Color = "white";
    axisHandle.XColor = "black";
    axisHandle.YColor = "black";
    axisHandle.GridColor = [0.75 0.75 0.75];
    axisHandle.Title.Color = "black";
    axisHandle.XLabel.Color = "black";
    axisHandle.YLabel.Color = "black";
end

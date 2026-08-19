clear;
clc;
close all;

paths = project_paths();
rng(0,"twister");

%% 1. Create and validate environment

env = create_cartpole_rl_environment();
validateEnvironment(env);
fprintf("Environment validation completed.\n");

%% 2. Reset environment

[observation,info] = reset(env);

fprintf("Initial normalized observation:\n");
disp(observation);

fprintf("Initial physical state:\n");
disp(info.State);

%% 3. Random-action rollout

maximumSteps = info.MaxSteps;
stateHistory = zeros(maximumSteps + 1,4);
actionHistory = zeros(maximumSteps,1);
rewardHistory = zeros(maximumSteps,1);
stateHistory(1,:) = info.State';

for stepIndex = 1:maximumSteps

    randomAction = -10 + 20*rand;

    [observation,reward,isDone,info] = ...
        step(env,randomAction);

    stateHistory(stepIndex + 1,:) = info.State';
    actionHistory(stepIndex) = randomAction;
    rewardHistory(stepIndex) = reward;

    if isDone
        break;
    end
end

%% 4. Trim state, action, and reward histories correctly

episodeSteps = info.StepCount;
stateHistory = stateHistory(1:episodeSteps + 1,:);
actionHistory = actionHistory(1:episodeSteps);
rewardHistory = rewardHistory(1:episodeSteps);

stateTime = (0:episodeSteps)'*info.Parameters.dt;
actionTime = (0:episodeSteps - 1)'*info.Parameters.dt;

%% 5. Print result

fprintf("Episode steps: %d\n",episodeSteps);
fprintf( ...
    "Episode time: %.3f s\n", ...
    episodeSteps*info.Parameters.dt);
fprintf("Failed: %d\n",info.TerminatedByFailure);
fprintf("Total reward: %.3f\n",sum(rewardHistory));

%% 6. Plot random-policy response

figure( ...
    "Name","Random Policy RL Environment Test", ...
    "Position",[100 100 1200 750]);

tiledlayout(3,1);

nexttile;
plot( ...
    stateTime, ...
    rad2deg(stateHistory(:,3)), ...
    "LineWidth",1.5);
hold on;
yline(rad2deg(info.Parameters.angle_limit),"r--");
yline(-rad2deg(info.Parameters.angle_limit),"r--");
grid on;
ylabel("Angle [deg]");
title("Pole Angle");

nexttile;
plot( ...
    stateTime, ...
    stateHistory(:,1), ...
    "LineWidth",1.5);
grid on;
ylabel("Position [m]");
title("Cart Position");

nexttile;
stairs( ...
    actionTime, ...
    actionHistory, ...
    "LineWidth",1.2);
grid on;
xlabel("Time [s]");
ylabel("Force [N]");
title("Random Action");

sgtitle("Custom Nonlinear CartPole: Random Policy Baseline");

outputPath = fullfile( ...
    paths.Images, ...
    "random_policy_baseline.png");

exportgraphics(gcf,outputPath,"Resolution",180);
fprintf("Saved: %s\n",outputPath);

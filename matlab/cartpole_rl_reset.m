function [initialObservation,info] = cartpole_rl_reset()
% CARTPOLE_RL_RESET Reset function for the custom RL environment.

p = cartpole_params();

%% Randomized initial state

initialPosition = ...
    -0.5 + 1.0*rand;

initialPositionVelocity = ...
    -0.2 + 0.4*rand;

initialAngle = deg2rad( ...
    -8.0 + 16.0*rand);

initialAngleVelocity = ...
    -0.3 + 0.6*rand;

initialState = [
    initialPosition
    initialPositionVelocity
    initialAngle
    initialAngleVelocity
    ];

%% Information passed between environment steps

info.State = initialState;
info.PreviousAction = 0;
info.StepCount = 0;
info.MaxSteps = round(10.0/p.dt);
info.Parameters = p;
info.TerminatedByFailure = false;

%% Observation delivered to the agent

initialObservation = ...
    normalize_cartpole_observation( ...
    initialState, ...
    p);
end
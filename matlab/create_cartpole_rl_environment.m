function env = create_cartpole_rl_environment()
% CREATE_CARTPOLE_RL_ENVIRONMENT Create custom continuous-action CartPole RL environment.

%% Observation specification

observationInfo = rlNumericSpec([4 1]);

observationInfo.Name = ...
    "normalized_cartpole_state";

observationInfo.Description = ...
    "normalized x, xDot, theta, thetaDot";

observationInfo.LowerLimit = ...
    -5*ones(4,1);

observationInfo.UpperLimit = ...
    5*ones(4,1);

%% Continuous action specification

actionInfo = rlNumericSpec([1 1]);

actionInfo.Name = ...
    "cart_force";

actionInfo.Description = ...
    "continuous horizontal force in Newtons";

actionInfo.LowerLimit = -10;
actionInfo.UpperLimit = 10;

%% Create function-based environment

env = rlFunctionEnv( ...
    observationInfo, ...
    actionInfo, ...
    @cartpole_rl_step, ...
    @cartpole_rl_reset);
end
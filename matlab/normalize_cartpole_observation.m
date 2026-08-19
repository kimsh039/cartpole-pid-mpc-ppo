function observation = normalize_cartpole_observation(state,p)
% NORMALIZE_CARTPOLE_OBSERVATION Scale CartPole states for neural networks.

observation = [
    state(1) / p.position_limit
    state(2) / 3.0
    state(3) / p.angle_limit
    state(4) / 3.0
    ];

% Prevent abnormally large neural-network inputs.
observation = min( ...
    max(observation,-5), ...
    5);
end
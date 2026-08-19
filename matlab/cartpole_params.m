function p = cartpole_params()
% CARTPOLE_PARAMS Physical and simulation parameters.

% Physical parameters
p.M = 1.0;                  % Cart mass [kg]
p.m = 0.1;                  % Pole mass [kg]
p.l = 0.5;                  % Pivot to pole center of mass [m]
p.g = 9.8;                  % Gravity [m/s^2]

% Simulation parameters
p.dt = 0.02;                % Control period [s]
p.duration = 5.0;           % Maximum simulation time [s]

% Safety and actuator limits
p.force_limit = 10.0;       % Maximum force magnitude [N]
p.position_limit = 2.4;     % Maximum cart position [m]
p.angle_limit = deg2rad(12); % Failure angle [rad]
end
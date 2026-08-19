function ds = cartpole_dynamics(~, s, force, p)
% CARTPOLE_DYNAMICS Nonlinear CartPole continuous-time dynamics.
%
% State:
%   s(1) = cart position       x         [m]
%   s(2) = cart velocity       x_dot     [m/s]
%   s(3) = pole angle          theta     [rad]
%   s(4) = pole angular rate   theta_dot [rad/s]
%
% Input:
%   force = horizontal force applied to cart [N]
%
% Convention:
%   theta = 0 means upright.

x_dot = s(2);
theta = s(3);
theta_dot = s(4);

% Actuator saturation
force = min(max(force, -p.force_limit), p.force_limit);

total_mass = p.M + p.m;

temp = ...
    (force + p.m*p.l*theta_dot^2*sin(theta)) ...
    / total_mass;

theta_ddot = ...
    (p.g*sin(theta) - cos(theta)*temp) ...
    / (p.l*(4/3 - p.m*cos(theta)^2/total_mass));

x_ddot = ...
    temp ...
    - p.m*p.l*theta_ddot*cos(theta)/total_mass;

ds = [
    x_dot
    x_ddot
    theta_dot
    theta_ddot
    ];
end
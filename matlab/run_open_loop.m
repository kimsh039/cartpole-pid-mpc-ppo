clear;
clc;
close all;

p = cartpole_params();

% Initial state:
% x = 0 m
% x_dot = 0 m/s
% theta = 5 degrees
% theta_dot = 0 rad/s
s0 = [
    0
    0
    deg2rad(-5)
    0
];

% No controller
force = 0;

% Return results at the control sampling interval
time = 0:p.dt:p.duration;

% Stop when the pole falls or cart leaves the track
options = odeset( ...
    "Events", @(t,s) stopEvent(t,s,p), ...
    "RelTol", 1e-9, ...
    "AbsTol", 1e-10);

[t, state, te, se, ie] = ode45( ...
    @(t,s) cartpole_dynamics(t,s,force,p), ...
    time, ...
    s0, ...
    options);

x = state(:,1);
xDot = state(:,2);
theta = state(:,3);
thetaDot = state(:,4);

fprintf("Initial pole angle: %.3f deg\n", rad2deg(theta(1)));
fprintf("Final simulation time: %.3f s\n", t(end));
fprintf("Final pole angle: %.3f deg\n", rad2deg(theta(end)));
fprintf("Maximum cart displacement: %.4f m\n", max(abs(x)));

if ~isempty(ie)
    if ie(end) == 1
        fprintf("Termination reason: pole angle exceeded %.1f deg\n", ...
            rad2deg(p.angle_limit));
    elseif ie(end) == 2
        fprintf("Termination reason: cart position exceeded %.1f m\n", ...
            p.position_limit);
    end
else
    fprintf("Termination reason: time limit reached\n");
end

figure("Name","Open-loop CartPole response");

tiledlayout(2,2);

nexttile;
plot(t,x,"LineWidth",1.5);
yline(0,"--");
grid on;
xlabel("Time [s]");
ylabel("Cart position [m]");
title("Cart Position");

nexttile;
plot(t,xDot,"LineWidth",1.5);
yline(0,"--");
grid on;
xlabel("Time [s]");
ylabel("Cart velocity [m/s]");
title("Cart Velocity");

nexttile;
plot(t,rad2deg(theta),"LineWidth",1.5);
yline(rad2deg(p.angle_limit),"r--","Failure limit");
yline(-rad2deg(p.angle_limit),"r--");
grid on;
xlabel("Time [s]");
ylabel("Pole angle [deg]");
title("Pole Angle");

nexttile;
plot(t,rad2deg(thetaDot),"LineWidth",1.5);
yline(0,"--");
grid on;
xlabel("Time [s]");
ylabel("Angular velocity [deg/s]");
title("Pole Angular Velocity");

sgtitle("Uncontrolled Nonlinear CartPole");

function [value, isterminal, direction] = stopEvent(~,s,p)
    angleRemaining = p.angle_limit - abs(s(3));
    positionRemaining = p.position_limit - abs(s(1));

    value = [
        angleRemaining
        positionRemaining
    ];

    % Stop integration when either value reaches zero
    isterminal = [1; 1];

    % Detect positive-to-negative crossing
    direction = [-1; -1];
end

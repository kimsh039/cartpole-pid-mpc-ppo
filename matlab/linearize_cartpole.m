clear;
clc;
close all;

paths = project_paths();

p = cartpole_params();

%% 1. Symbolic variables

syms x xDot theta thetaDot force real
syms M m l g positive

state = [
    x
    xDot
    theta
    thetaDot
];

%% 2. Nonlinear CartPole dynamics

totalMass = M + m;

temp = ...
    (force + m*l*thetaDot^2*sin(theta)) ...
    / totalMass;

thetaDDot = ...
    (g*sin(theta) - cos(theta)*temp) ...
    / (l*(4/3 - m*cos(theta)^2/totalMass));

xDDot = ...
    temp ...
    - m*l*thetaDDot*cos(theta)/totalMass;

f = [
    xDot
    xDDot
    thetaDot
    thetaDDot
];

fprintf("Nonlinear state derivative:\n");
disp(f);

%% 3. Compute symbolic Jacobians

A_symbolic = simplify(jacobian(f,state));
B_symbolic = simplify(jacobian(f,force));

fprintf("Symbolic A matrix:\n");
disp(A_symbolic);

fprintf("Symbolic B matrix:\n");
disp(B_symbolic);

%% 4. Substitute upright equilibrium and physical parameters

variables = [
    x
    xDot
    theta
    thetaDot
    force
    M
    m
    l
    g
];

values = [
    0
    0
    0
    0
    0
    p.M
    p.m
    p.l
    p.g
];

A = double(subs(A_symbolic,variables,values));
B = double(subs(B_symbolic,variables,values));

C = eye(4);
D = zeros(4,1);

fprintf("Numerical continuous-time A matrix:\n");
disp(A);

fprintf("Numerical continuous-time B matrix:\n");
disp(B);

%% 5. Open-loop stability analysis

continuousEigenvalues = eig(A);

fprintf("Continuous-time eigenvalues:\n");
disp(continuousEigenvalues);

if any(real(continuousEigenvalues) > 0)
    fprintf("RESULT: The upright equilibrium is OPEN-LOOP UNSTABLE.\n");
else
    fprintf("RESULT: No unstable continuous-time eigenvalue found.\n");
end

%% 6. Controllability analysis

controllabilityMatrix = ctrb(A,B);
controllabilityRank = rank(controllabilityMatrix);
numberOfStates = size(A,1);

fprintf("Controllability rank: %d / %d\n", ...
    controllabilityRank,numberOfStates);

if controllabilityRank == numberOfStates
    fprintf("RESULT: The linearized CartPole is controllable.\n");
else
    fprintf("RESULT: The linearized CartPole is not fully controllable.\n");
end

%% 7. Convert to discrete-time model

continuousSystem = ss(A,B,C,D);

discreteSystem = c2d( ...
    continuousSystem, ...
    p.dt, ...
    "zoh");

Ad = discreteSystem.A;
Bd = discreteSystem.B;

fprintf("Discrete-time Ad matrix:\n");
disp(Ad);

fprintf("Discrete-time Bd matrix:\n");
disp(Bd);

discreteEigenvalues = eig(Ad);

fprintf("Discrete-time eigenvalues:\n");
disp(discreteEigenvalues);

if any(abs(discreteEigenvalues) > 1)
    fprintf("RESULT: The discrete open-loop system is unstable.\n");
end

%% 8. Save model for PID and MPC

save( ...
    fullfile(paths.Models,"cartpole_linear_model.mat"), ...
    "A", ...
    "B", ...
    "C", ...
    "D", ...
    "Ad", ...
    "Bd", ...
    "p");

fprintf("Saved: %s\n", ...
    fullfile(paths.Models,"cartpole_linear_model.mat"));

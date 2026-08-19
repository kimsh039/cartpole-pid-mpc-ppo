clear;
clc;
close all;

paths = project_paths();

%% 1. Parameters

p = cartpole_params();

dt = p.dt;
duration = 10.0;
numberOfSteps = round(duration/dt);

%% 2. Reference state

positionReference = 0.0;
angleReferenceLimit = deg2rad(8);

%% 3. Initial state

initialState = [
    0.20             % Cart position [m]
    0.00             % Cart velocity [m/s]
    deg2rad(5)       % Pole angle [rad]
    0.00             % Pole angular velocity [rad/s]
];

%% 4. Outer position PID gains

KpPosition = 0.12;
KiPosition = 0.02;
KdPosition = 0.16;

%% 5. Inner angle PID gains

KpAngle = 60.0;
KiAngle = 1.0;
KdAngle = 15.0;

%% 6. Anti-windup integral limits

positionIntegralLimit = 1.0;
angleIntegralLimit = 0.30;

%% 7. Allocate result arrays

time = (0:numberOfSteps)'*dt;

stateHistory = zeros(numberOfSteps + 1,4);
forceHistory = zeros(numberOfSteps + 1,1);
angleReferenceHistory = zeros(numberOfSteps + 1,1);

stateHistory(1,:) = initialState';

positionIntegral = 0;
angleIntegral = 0;

lastIndex = numberOfSteps + 1;
failureReason = "none";

%% 8. Digital control loop

for step = 1:numberOfSteps

    currentState = stateHistory(step,:)';

    position = currentState(1);
    positionVelocity = currentState(2);
    angle = currentState(3);
    angleVelocity = currentState(4);

    %% Outer position PID

    positionError = ...
        position - positionReference;

    positionIntegral = ...
        positionIntegral ...
        + positionError*dt;

    positionIntegral = min( ...
        max( ...
            positionIntegral, ...
            -positionIntegralLimit), ...
        positionIntegralLimit);

    rawAngleReference = -( ...
        KpPosition*positionError ...
        + KiPosition*positionIntegral ...
        + KdPosition*positionVelocity);

    angleReference = min( ...
        max( ...
            rawAngleReference, ...
            -angleReferenceLimit), ...
        angleReferenceLimit);

    %% Inner angle PID

    angleError = ...
        angle - angleReference;

    angleIntegral = ...
        angleIntegral ...
        + angleError*dt;

    angleIntegral = min( ...
        max( ...
            angleIntegral, ...
            -angleIntegralLimit), ...
        angleIntegralLimit);

    rawForce = ...
        KpAngle*angleError ...
        + KiAngle*angleIntegral ...
        + KdAngle*angleVelocity;

    %% Actuator saturation

    appliedForce = min( ...
        max( ...
            rawForce, ...
            -p.force_limit), ...
        p.force_limit);

    %% Advance nonlinear plant by one control interval

    nextState = rk4_step( ...
        currentState, ...
        appliedForce, ...
        dt, ...
        p);

    %% Save data

    stateHistory(step + 1,:) = nextState';
    forceHistory(step) = appliedForce;
    angleReferenceHistory(step) = angleReference;

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

%% 9. Trim unused data

time = time(1:lastIndex);
stateHistory = stateHistory(1:lastIndex,:);
forceHistory = forceHistory(1:lastIndex);
angleReferenceHistory = ...
    angleReferenceHistory(1:lastIndex);

if lastIndex > 1
    forceHistory(end) = forceHistory(end - 1);
    angleReferenceHistory(end) = ...
        angleReferenceHistory(end - 1);
end

%% 10. Extract states

position = stateHistory(:,1);
positionVelocity = stateHistory(:,2);
angle = stateHistory(:,3);
angleVelocity = stateHistory(:,4);

%% 11. Calculate performance metrics

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

success = ...
    lastIndex == numberOfSteps + 1;

fprintf("\n=== PID RESULT ===\n");
fprintf("Success: %d\n",success);
fprintf("Failure reason: %s\n",failureReason);
fprintf("Simulation time: %.2f s\n",time(end));
fprintf("Angle RMSE: %.4f deg\n",angleRMSEDeg);
fprintf("Position RMSE: %.4f m\n",positionRMSE);
fprintf("Maximum angle: %.4f deg\n",maximumAngleDeg);
fprintf("Maximum position: %.4f m\n",maximumPosition);
fprintf("Control effort: %.4f N^2*s\n",controlEnergy);

%% 12. Plot results

figure("Name","Cascade PID CartPole");

tiledlayout(3,1);

nexttile;

plot( ...
    time, ...
    rad2deg(angle), ...
    "b-", ...
    "LineWidth",1.6);

hold on;

plot( ...
    time, ...
    rad2deg(angleReferenceHistory), ...
    "r--", ...
    "LineWidth",1.3);

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
legend( ...
    "Actual angle", ...
    "Reference angle", ...
    "Location","best");

nexttile;

plot( ...
    time, ...
    position, ...
    "LineWidth",1.6);

hold on;
yline(positionReference,"r--");

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
xlabel("Time [s]");
ylabel("Force [N]");
title("Control Force");

sgtitle("Nonlinear CartPole with Cascade PID");

%% 13. Save result

resultTable = table( ...
    time, ...
    position, ...
    positionVelocity, ...
    angle, ...
    angleVelocity, ...
    angleReferenceHistory, ...
    forceHistory, ...
    VariableNames=[ ...
        "time", ...
        "position", ...
        "positionVelocity", ...
        "angle", ...
        "angleVelocity", ...
        "angleReference", ...
        "force" ...
    ]);

writetable( ...
    resultTable, ...
    fullfile(paths.Trajectories,"pid_result.csv"));

fprintf("Saved: %s\n", ...
    fullfile(paths.Trajectories,"pid_result.csv"));

function [nextObservation,reward,isDone,updatedInfo] = ...
    cartpole_rl_step(action,info)
% CARTPOLE_RL_STEP One interaction step of the custom CartPole RL environment.

    p = info.Parameters;
    currentState = info.State;

    %% 1. Convert and constrain action

    appliedForce = double(action(1));

    appliedForce = min( ...
        max( ...
            appliedForce, ...
            -p.force_limit), ...
        p.force_limit);

    %% 2. Advance nonlinear CartPole dynamics

    nextState = rk4_step( ...
        currentState, ...
        appliedForce, ...
        p.dt, ...
        p);

    %% 3. Construct normalized observation

    nextObservation = ...
        normalize_cartpole_observation( ...
            nextState, ...
            p);

    %% 4. Calculate reward

    normalizedState = nextObservation;

    stateWeights = diag([
        1.0
        0.1
        4.0
        0.1
    ]);

    stateCost = ...
        normalizedState' ...
        * stateWeights ...
        * normalizedState;

    normalizedAction = ...
        appliedForce / p.force_limit;

    actionCost = ...
        0.05*normalizedAction^2;

    normalizedActionChange = ...
        (appliedForce - info.PreviousAction) ...
        / p.force_limit;

    actionRateCost = ...
        0.02*normalizedActionChange^2;

    totalCost = ...
        stateCost ...
        + actionCost ...
        + actionRateCost;

    reward = exp(-totalCost);

    %% 5. Check failure conditions

    angleFailure = ...
        abs(nextState(3)) > p.angle_limit;

    positionFailure = ...
        abs(nextState(1)) > p.position_limit;

    numericalFailure = ...
        any(~isfinite(nextState));

    failed = ...
        angleFailure ...
        || positionFailure ...
        || numericalFailure;

    if failed
        reward = reward - 10;
    end

    %% 6. Update step count

    nextStepCount = info.StepCount + 1;

    timeLimitReached = ...
        nextStepCount >= info.MaxSteps;

    isDone = ...
        failed ...
        || timeLimitReached;

    %% 7. Update environment information

    updatedInfo = info;

    updatedInfo.State = nextState;
    updatedInfo.PreviousAction = appliedForce;
    updatedInfo.StepCount = nextStepCount;
    updatedInfo.TerminatedByFailure = failed;
end

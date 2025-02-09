package com.adambots.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class StateMachine2<S extends Enum<S>, P> {
    private S currentState;
    private S targetState;  // New: track target state separately
    private P targetProperties;
    private final Consumer<String> logger;
    private final boolean usePositionControl;  // New: flag for position control mode
    private BooleanSupplier atTargetCheck;    // New: store the at-target check

    public StateMachine2(S initialState, P initialProperties, Consumer<String> logger, boolean usePositionControl) {
        this.currentState = initialState;
        this.targetState = initialState;
        this.targetProperties = initialProperties;
        this.logger = logger != null ? logger : (msg -> {});
        this.usePositionControl = usePositionControl;
    }

    public S getCurrentState() {
        return currentState;
    }

    public P getTargetProperties() {
        return targetProperties;
    }

    public void requestTransition(S newState, P properties, BooleanSupplier atTarget, Consumer<P> action) {
        if (newState != targetState) {
            logger.accept("Transitioning from " + currentState + " to " + newState);
            this.targetState = newState;
            this.targetProperties = properties;
            this.atTargetCheck = atTarget;
            action.accept(properties);
            
            if (usePositionControl) {
                // For position control, update state immediately
                currentState = newState;
            }
        }
    }

    public void periodic() {
        if (!usePositionControl && targetState != currentState) {
            // For non-position control, check if target is reached
            if (atTargetCheck != null && atTargetCheck.getAsBoolean()) {
                logger.accept("Reached target state: " + targetState);
                currentState = targetState;
            }
        }
    }
}
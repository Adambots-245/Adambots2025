package com.adambots.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class StateMachine<T> {
    private final T context;
    private final Map<String, State> states = new HashMap<>();
    private State currentState = null;
    private final String name;
    private final Consumer<String> logger;

    // Transition tracking
    private State targetState = null;
    private TransitionData activeTransition = null;

    public StateMachine(String name, T context, Consumer<String> logger) {
        this.name = name;
        this.context = context;
        this.logger = logger != null ? logger : (msg -> {});
    }

    private void log(String message) {
        logger.accept(String.format("[StateMachine-%s] %s", name, message));
    }

    public State getCurrentState() {
        return currentState;
    }

    public T getContext() {
        return context;
    }

    private class TransitionData {
        final Consumer<T> action;
        final Consumer<T> onComplete;
        final State fromState;
        final State toState;

        TransitionData(State from, State to, Consumer<T> action, Consumer<T> onComplete) {
            this.fromState = from;
            this.toState = to;
            this.action = action;
            this.onComplete = onComplete;
        }
    }

    public class State {
        private final String name;
        private final BooleanSupplier trigger;
        private final Map<State, TransitionData> transitions = new HashMap<>();
        
        private State(String name, BooleanSupplier trigger) {
            this.name = name;
            this.trigger = trigger;
        }

        public String getName() {
            return name;
        }

        public boolean isTriggered() {
            return trigger.getAsBoolean();
        }

        public void addTransition(State targetState, Consumer<T> action, Consumer<T> onComplete) {
            TransitionData transition = new TransitionData(this, targetState, action, onComplete);
            transitions.put(targetState, transition);
            log(String.format("Added transition from %s to %s", name, targetState.getName()));
        }
    }

    public State addState(String name, BooleanSupplier trigger) {
        if (states.containsKey(name)) {
            throw new IllegalArgumentException("State " + name + " already exists");
        }

        State state = new State(name, trigger);
        states.put(name, state);
        
        if (currentState == null) {
            currentState = state;
            log("Initial state set to " + name);
        }

        return state;
    }

    public void requestTransition(State targetState) {
        if (targetState == null || currentState == null) {
            log("Invalid transition request: null state");
            return;
        }

        if (activeTransition != null) {
            log("Transition already in progress");
            return;
        }

        TransitionData transition = currentState.transitions.get(targetState);
        if (transition != null) {
            log(String.format("Starting transition from %s to %s", 
                currentState.getName(), targetState.getName()));
            this.targetState = targetState;
            this.activeTransition = transition;
            
            if (transition.action != null) {
                transition.action.accept(context);
            }
        } else {
            log(String.format("No transition defined from %s to %s", 
                currentState.getName(), targetState.getName()));
        }
    }

    public void periodic() {
        if (activeTransition != null && targetState != null) {
            // Check if target state is reached
            if (targetState.isTriggered()) {
                log(String.format("Reached target state %s", targetState.getName()));
                
                // Execute completion handler
                if (activeTransition.onComplete != null) {
                    log("Executing completion handler");
                    activeTransition.onComplete.accept(context);
                }
                
                // Update current state
                currentState = targetState;
                
                // Clear transition data
                activeTransition = null;
                targetState = null;
                
                log(String.format("Completed transition to %s", currentState.getName()));
            }
        }
    }
}
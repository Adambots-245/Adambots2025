package com.adambots.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.ArrayList;
import java.util.List;

public class StateMachine<T> {
    private final T context;
    private final Map<String, State> states = new HashMap<>();
    private State currentState = null;
    private final List<String> stateLog = new ArrayList<>();
    private boolean debugEnabled = false;

    public StateMachine(T context) {
        this.context = context;
    }

    public void setDebug(boolean enabled) {
        this.debugEnabled = enabled;
    }

    public void log(String message) {
        if (debugEnabled) {
            System.out.println("[StateMachine] " + message);
            stateLog.add(message);
        }
    }

    public List<String> getStateLog() {
        return new ArrayList<>(stateLog);
    }

    public State getCurrentState() {
        return currentState;
    }

    public T getContext() {
        return context;
    }

    public class State {
        private final String name;
        private final BooleanSupplier trigger;
        private final Map<State, Consumer<T>> transitionActions = new HashMap<>();
        
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

        public void addTransition(State targetState, Consumer<T> action) {
            transitionActions.put(targetState, action);
            log("Added transition from " + name + " to " + targetState.getName());
        }

        public boolean hasTransitionTo(State targetState) {
            return transitionActions.containsKey(targetState);
        }
    }

    public State addState(String name, BooleanSupplier trigger) {
        // Validate state name uniqueness
        if (states.containsKey(name)) {
            throw new IllegalArgumentException("State " + name + " already exists");
        }

        // Create and store the new state
        State state = new State(name, trigger);
        states.put(name, state);
        
        // Set as initial state if this is the first state
        if (currentState == null) {
            currentState = state;
            log("Initial state set to " + name);
        }

        return state;
    }

    public void requestTransition(State targetState) {
        if (targetState == null) {
            log("Invalid target state: null");
            return;
        }

        if (currentState == null) {
            log("No current state set");
            return;
        }

        // Check if transition is defined
        Consumer<T> transitionAction = currentState.transitionActions.get(targetState);
        if (transitionAction != null) {
            log("Executing transition from " + currentState.getName() + 
                " to " + targetState.getName());
            transitionAction.accept(context);
        } else {
            log("No transition defined from " + currentState.getName() + 
                " to " + targetState.getName());
        }
    }

    public void periodic() {
        // Check for trigger condition conflicts
        for (State state : states.values()) {
            if (state != currentState && state.isTriggered()) {
                if (currentState.isTriggered()) {
                    log("Warning: Ambiguous state triggers detected between " + 
                        currentState.getName() + " and " + state.getName());
                }
            }
        }

        // Update current state based on triggers
        for (State state : states.values()) {
            if (state.isTriggered()) {
                if (state != currentState) {
                    log("State changed from " + 
                        (currentState != null ? currentState.getName() : "null") + 
                        " to " + state.getName() + " via trigger");
                    currentState = state;
                }
                break;
            }
        }
    }
}
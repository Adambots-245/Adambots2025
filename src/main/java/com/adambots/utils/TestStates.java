package com.adambots.utils;

import java.util.Scanner;

public class TestStates {
    public static class ArmContext {
        public double position = 0;
        public double targetPosition = 0;
        public double motorSpeed = 0;
        
        @Override
        public String toString() {
            return String.format("Position: %.1f, Target: %.1f, Speed: %.1f", 
                position, targetPosition, motorSpeed);
        }
    }

    public static void main(String[] args) throws InterruptedException {
        // Create context and state machine
        ArmContext context = new ArmContext();
        StateMachine<ArmContext> stateMachine = new StateMachine<>(context);
        stateMachine.setDebug(true);

        // Define states with their trigger conditions
        StateMachine<ArmContext>.State groundState = 
            stateMachine.addState("Ground", () -> Math.abs(context.position) < 5.0);

        StateMachine<ArmContext>.State midState = 
            stateMachine.addState("Mid", () -> Math.abs(context.position - 45.0) < 5.0);

        StateMachine<ArmContext>.State highState = 
            stateMachine.addState("High", () -> Math.abs(context.position - 90.0) < 5.0);

        // Define transitions and their actions
        groundState.addTransition(midState, ctx -> {
            ctx.targetPosition = 45.0;
            ctx.motorSpeed = 0.5;
        });

        midState.addTransition(highState, ctx -> {
            ctx.targetPosition = 90.0;
            ctx.motorSpeed = 0.5;
        });

        midState.addTransition(groundState, ctx -> {
            ctx.targetPosition = 0.0;
            ctx.motorSpeed = -0.5;
        });

        highState.addTransition(midState, ctx -> {
            ctx.targetPosition = 45.0;
            ctx.motorSpeed = -0.5;
        });

        // Start command line interface
        Scanner scanner = new Scanner(System.in);
        boolean running = true;

        System.out.println("State Machine Test Started");
        System.out.println("Commands: ground, mid, high, status, quit");

        while (running) {
            // Update position based on motor speed
            while (Math.abs(context.position - context.targetPosition) > 0.1) {
                context.position += context.motorSpeed;
            } 
            
            context.motorSpeed = 0;

            // Update state machine
            stateMachine.periodic();

            // Display current status
            System.out.printf("\nCurrent State: %s\n", 
                stateMachine.getCurrentState().getName());
            System.out.println("Context: " + context);

            // Get command
            System.out.print("\nEnter command: ");
            String command = scanner.nextLine().trim().toLowerCase();

            switch (command) {
                case "ground":
                    stateMachine.requestTransition(groundState);
                    break;
                case "mid":
                    stateMachine.requestTransition(midState);
                    break;
                case "high":
                    stateMachine.requestTransition(highState);
                    break;
                case "status":
                    System.out.println("\nRecent state machine logs:");
                    for (String log : stateMachine.getStateLog()) {
                        System.out.println(log);
                    }
                    break;
                case "quit":
                    running = false;
                    break;
                default:
                    System.out.println("Unknown command. Use: ground, mid, high, status, or quit");
            }

            // Small delay to simulate periodic updates
            Thread.sleep(100);
        }

        scanner.close();
        System.out.println("Test ended");
    }
}
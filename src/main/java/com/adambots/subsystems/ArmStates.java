package com.adambots.subsystems;

import java.util.Scanner;

public class ArmStates {
        public double elevatorPosition = 0;
        public double elevatorTargetPosition = 0;
        public double elevatorMotorSpeed = 0;
        
        public double wristPosition = 0;
        public double wristTargetPosition = 0;
        public double wristMotorSpeed = 0;

        @Override
        public String toString() {
            return String.format("ElevatorPosition: %.1f, ElevatorTarget: %.1f, ElevatorSpeed: %.1f, WristPosition %.1f, WristTarget %.1f, WristMotorSpeed %.1f", 
            elevatorPosition, elevatorTargetPosition, elevatorMotorSpeed, wristPosition, wristTargetPosition, wristMotorSpeed);
        }
    }
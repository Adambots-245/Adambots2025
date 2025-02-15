package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.adambots.utils.StateMachine2;

public class ElevatorPhotoSubsystem extends SubsystemBase {
    
    public record ElevatorProperties(
        double speed,            // Speed to move
        String description       // Description of state
    ) {}

    public enum ElevatorState {
        INTAKE(new ElevatorProperties(-0.5, "Intake Position")),
        MID(new ElevatorProperties(0.5, "Mid Position")),
        HIGH(new ElevatorProperties(0.5, "High Position"));

        public final ElevatorProperties properties;
        ElevatorState(ElevatorProperties properties) {
            this.properties = properties;
        }
    }

    // Hardware
    private final PWMSparkMax motor;
    private final DigitalInput intakePhotoEye;
    private final DigitalInput midPhotoEye;
    private final DigitalInput highPhotoEye;

    // State Machine
    private final StateMachine2<ElevatorState, ElevatorProperties> stateMachine;

    public ElevatorPhotoSubsystem() {
        // Initialize hardware
        motor = new PWMSparkMax(0);
        intakePhotoEye = new DigitalInput(0);
        midPhotoEye = new DigitalInput(1);
        highPhotoEye = new DigitalInput(2);

        // Initialize state machine with positional control OFF
        stateMachine = new StateMachine2<>(
            ElevatorState.INTAKE,
            ElevatorState.INTAKE.properties,
            message -> System.out.println("Elevator: " + message),
            false  // Not using position control
        );
    }

    private void setMotorOutput(ElevatorProperties properties) {
        motor.set(properties.speed());
    }

    private boolean isAtTarget() {
        // Check appropriate PhotoEye based on target state
        return switch(stateMachine.getCurrentState()) {
            case INTAKE -> !intakePhotoEye.get();  // Most PhotoEyes are active LOW
            case MID -> !midPhotoEye.get();
            case HIGH -> !highPhotoEye.get();
        };
    }

    @Override
    public void periodic() {
        // StateMachine will check PhotoEyes and update state
        stateMachine.periodic();
        
        // Update motor output based on current target
        setMotorOutput(stateMachine.getTargetProperties());
    }

    // Public methods for commanding the elevator
    public void moveToIntake() {
        stateMachine.requestTransition(
            ElevatorState.INTAKE,
            ElevatorState.INTAKE.properties,
            this::isAtTarget,
            this::setMotorOutput
        );
    }

    public void moveToMid() {
        stateMachine.requestTransition(
            ElevatorState.MID,
            ElevatorState.MID.properties,
            this::isAtTarget,
            this::setMotorOutput
        );
    }

    public void moveToHigh() {
        stateMachine.requestTransition(
            ElevatorState.HIGH,
            ElevatorState.HIGH.properties,
            this::isAtTarget,
            this::setMotorOutput
        );
    }
}
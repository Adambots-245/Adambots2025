package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.utils.StateMachine;

public class ElevatorSubsystem extends SubsystemBase {

    // Properties for Elevator
    public record ElevatorProperties(
            double heightInches,
            String description
    ) {}

    // Properties for Wrist
    public record WristProperties(
            double angleDegrees,
            String description
    ) {}

    // Define elevator states
    public enum ElevatorState {
        INTAKE(new ElevatorProperties(0.0, "Intake Position")),
        L1(new ElevatorProperties(12.0, "Level 1")),
        L2(new ElevatorProperties(24.0, "Level 2")),
        L3(new ElevatorProperties(36.0, "Level 3")),
        L4(new ElevatorProperties(48.0, "Level 4"));

        public final ElevatorProperties properties;
        ElevatorState(ElevatorProperties properties) {
            this.properties = properties;
        }
    }

    // Define wrist states
    public enum WristState {
        STOWED(new WristProperties(0.0, "Stowed")),
        INTAKE(new WristProperties(20.0, "Intake Ready")),
        L1(new WristProperties(40.0, "Level 1 Angle")),
        L2(new WristProperties(60.0, "Level 2 Angle")),
        L3(new WristProperties(80.0, "Level 3 Angle")),
        L4(new WristProperties(100.0, "Level 4 Angle"));

        public final WristProperties properties;
        WristState(WristProperties properties) {
            this.properties = properties;
        }
    }

    // Hardware
    private final BaseMotor wristMotor;
    private final BaseMotor elevatorMotor;
    private final BaseAbsoluteEncoder wristEncoder;
    private final PIDController wristPID;

    // State Machines
    private final StateMachine<ElevatorState, ElevatorProperties> elevatorStateMachine;
    private final StateMachine<WristState, WristProperties> wristStateMachine;

    // Constants
    private static final double GEAR_RATIO = 10.0;
    private static final double DRUM_CIRCUMFERENCE = 2.0; // inches
    private static final double INCHES_PER_ROTATION = DRUM_CIRCUMFERENCE / GEAR_RATIO;
    private static final double ELEVATOR_POSITION_TOLERANCE = 0.5; // inches
    private static final double WRIST_POSITION_TOLERANCE = 2.0; // degrees

    public ElevatorSubsystem(BaseMotor elevatorMotor, BaseMotor wristMotor, BaseAbsoluteEncoder wristEncoder) {
        this.wristMotor = wristMotor;
        this.elevatorMotor = elevatorMotor;
        this.wristEncoder = wristEncoder;
        
        // Initialize PID
        this.wristPID = new PIDController(0.5, 0, 0);
        this.wristPID.setTolerance(WRIST_POSITION_TOLERANCE);

        configureMotors();

        // Initialize elevator state machine with position control
        elevatorStateMachine = new StateMachine<>(
            ElevatorState.INTAKE,
            ElevatorState.INTAKE.properties,
            message -> SmartDashboard.putString("Elevator/Status", message),
            true  // Using position control
        );

        // Initialize wrist state machine without position control
        wristStateMachine = new StateMachine<>(
            WristState.STOWED,
            WristState.STOWED.properties,
            message -> SmartDashboard.putString("Wrist/Status", message),
            false  // Not using position control
        );
    }

    private void configureMotors() {
        // Configure elevator motor
        elevatorMotor.setPID(0, 0.5, 0.0, 0.0, 0.0);
        elevatorMotor.setBrakeMode(true);

        // Configure wrist motor
        wristMotor.setBrakeMode(true);
    }

    private void setElevatorPosition(ElevatorProperties properties) {
        double rotations = properties.heightInches() / INCHES_PER_ROTATION;
        elevatorMotor.setPosition(rotations);
    }

    private void setWristOutput(WristProperties properties) {
        double output = wristPID.calculate(wristEncoder.getAbsolutePositionDegrees(), 
                                         properties.angleDegrees());
        if (wristPID.atSetpoint()) {
            output = 0;
        }
        wristMotor.set(output);
    }

    private boolean isWristAtTarget() {
        return wristPID.atSetpoint();
    }

    @Override
    public void periodic() {
        // Get current positions
        double currentHeight = elevatorMotor.getPosition() * INCHES_PER_ROTATION;
        double currentAngle = wristEncoder.getAbsolutePositionDegrees();

        // Update wrist state machine
        wristStateMachine.periodic();

        // Update dashboard
        SmartDashboard.putNumber("Elevator/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("Elevator/TargetHeight", 
            elevatorStateMachine.getTargetProperties().heightInches());
        SmartDashboard.putString("Elevator/State", 
            elevatorStateMachine.getCurrentState().toString());

        SmartDashboard.putNumber("Wrist/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("Wrist/TargetAngle", 
            wristStateMachine.getTargetProperties().angleDegrees());
        SmartDashboard.putString("Wrist/State", 
            wristStateMachine.getCurrentState().toString());
        SmartDashboard.putBoolean("Wrist/AtTarget", isWristAtTarget());
    }

    // Public methods for commanding the elevator
    public void moveElevatorToState(ElevatorState state) {
        elevatorStateMachine.requestTransition(
            state,
            state.properties,
            () -> true,  // No check needed for position control
            this::setElevatorPosition
        );
    }

    // Public methods for commanding the wrist
    public void moveWristToState(WristState state) {
        wristStateMachine.requestTransition(
            state,
            state.properties,
            this::isWristAtTarget,
            this::setWristOutput
        );
    }

    // Methods to check states
    public boolean isWristAtState(WristState state) {
        return wristStateMachine.getCurrentState() == state && isWristAtTarget();
    }

    public ElevatorState getCurrentElevatorState() {
        return elevatorStateMachine.getCurrentState();
    }

    public WristState getCurrentWristState() {
        return wristStateMachine.getCurrentState();
    }

    
}
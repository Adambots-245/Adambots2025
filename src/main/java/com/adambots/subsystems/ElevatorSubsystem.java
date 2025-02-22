package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.utils.StateMachine;
import com.adambots.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    // Properties for Elevator State
    public record ElevatorProperties(
            double heightInches,
            String description) {
    }

    // Properties for Wrist State
    public record WristProperties(
            double angleDegrees,
            String description) {
    }

    // Define elevator states
    public enum ElevatorState {
        INTAKE(new ElevatorProperties(ElevatorConstants.kElevatorIntakePosition, "Intake Position")),
        L1(new ElevatorProperties(ElevatorConstants.kElevatorL1Position, "Level 1")),
        L2(new ElevatorProperties(ElevatorConstants.kElevatorL2Position, "Level 2")),
        L3(new ElevatorProperties(ElevatorConstants.kElevatorL3Position, "Level 3")),
        L4(new ElevatorProperties(ElevatorConstants.kElevatorL4Position, "Level 4"));

        public final ElevatorProperties properties;

        ElevatorState(ElevatorProperties properties) {
            this.properties = properties;
        }
    }

    // Define wrist states
    public enum WristState {
        STOWED(new WristProperties(ElevatorConstants.kWristStowedPosition, "Stowed")),
        INTAKE(new WristProperties(ElevatorConstants.kWristIntakePosition, "Intake Ready")),
        L1(new WristProperties(ElevatorConstants.kWristL1Position, "Level 1 Angle")),
        L2(new WristProperties(ElevatorConstants.kWristL2Position, "Level 2 Angle")),
        L3(new WristProperties(ElevatorConstants.kWristL3Position, "Level 3 Angle")),
        L4(new WristProperties(ElevatorConstants.kWristL4Position, "Level 4 Angle"));

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

    public ElevatorSubsystem(BaseMotor elevatorMotor, BaseMotor wristMotor, BaseAbsoluteEncoder wristEncoder) {
        this.wristMotor = wristMotor;
        this.elevatorMotor = elevatorMotor;
        this.wristEncoder = wristEncoder;

        // Initialize PID
        this.wristPID = new PIDController(ElevatorConstants.kPWristController, ElevatorConstants.kIWristController,
                ElevatorConstants.kDWristController);
        this.wristPID.setTolerance(ElevatorConstants.kWristPositionTolerance);

        configureMotors();

        // Initialize elevator state machine with position control
        elevatorStateMachine = new StateMachine<>(
                ElevatorState.INTAKE,
                ElevatorState.INTAKE.properties,
                message -> SmartDashboard.putString("Elevator/Status", message),
                true // Using position control
        );

        // Initialize wrist state machine without position control
        wristStateMachine = new StateMachine<>(
                WristState.STOWED,
                WristState.STOWED.properties,
                message -> SmartDashboard.putString("Wrist/Status", message),
                false // Not using position control
        );
    }

    private void configureMotors() {
        // Configure elevator motor
        elevatorMotor.setPID(ElevatorConstants.kPIDSlot, ElevatorConstants.kPElevatorController,
                ElevatorConstants.kIElevatorController, ElevatorConstants.kDElevatorController,
                ElevatorConstants.kFElevatorController);
        elevatorMotor.setBrakeMode(true);

        // Configure wrist motor
        wristMotor.setBrakeMode(true);
    }

    private void setElevatorPosition(ElevatorProperties properties) {
        double rotations = properties.heightInches() / ElevatorConstants.kInchesPerRotation;
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
        double currentHeight = elevatorMotor.getPosition() * ElevatorConstants.kInchesPerRotation;
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
                () -> true, // No check needed for position control
                this::setElevatorPosition);
    }

    // Public methods for commanding the wrist
    public void moveWristToState(WristState state) {
        wristStateMachine.requestTransition(
                state,
                state.properties,
                this::isWristAtTarget,
                this::setWristOutput);
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
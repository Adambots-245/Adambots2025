package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.adambots.actuators.BaseMotor;
import com.adambots.utils.StateMachine;
import com.adambots.Constants.ElevatorConstants;
import com.adambots.RobotMap;

public class ElevatorSubsystem extends SubsystemBase {

    // Properties for Elevator State
    public record ElevatorProperties(
            double position,
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

    // Hardware
    private final BaseMotor elevatorMotor;

    double currentPosition;

    // State Machines
    private final StateMachine<ElevatorState, ElevatorProperties> elevatorStateMachine;

    public ElevatorSubsystem(BaseMotor elevatorMotor) {
        this.elevatorMotor = elevatorMotor;

        configureMotors();

        // Initialize elevator state machine with position control
        elevatorStateMachine = new StateMachine<>(
                ElevatorState.INTAKE,
                ElevatorState.INTAKE.properties,
                message -> SmartDashboard.putString("Elevator/Status", message),
                true // Using position control
        );
    }

    private void configureMotors() {
        // Configure elevator motor
        elevatorMotor.setPID(ElevatorConstants.kPIDSlot, ElevatorConstants.kPElevatorController,
                ElevatorConstants.kIElevatorController, ElevatorConstants.kDElevatorController,
                ElevatorConstants.kFElevatorController);
        elevatorMotor.setBrakeMode(false);
        elevatorMotor.setInverted(true);

        elevatorMotor.configureHardLimits(true,true, ElevatorConstants.kElevatorL4Position, 0);
    }

    private void setElevatorPosition(ElevatorProperties properties) {
        // double rotations = properties.position() / ElevatorConstants.kInchesPerRotation;
        double rotations = properties.position();

        setPosition(rotations);
    }

    private void setPosition(double rotations) {
        elevatorMotor.setPosition(rotations);
    }

    public double getCurrentPosition() {
        return elevatorMotor.getPosition();
    }

    public void holdElevatorPosition() {
        setPosition(currentPosition);
    }

    @Override
    public void periodic() {

        // Get current positions
        // currentPosition = elevatorMotor.getPosition() * ElevatorConstants.kInchesPerRotation;
        currentPosition = elevatorMotor.getPosition();


        // Update elevator state machine
        elevatorStateMachine.periodic();

        SmartDashboard.putNumber("Elevator/Encoder", elevatorMotor.getPosition());
        // Update dashboard
        SmartDashboard.putNumber("Elevator/CurrentPosition", currentPosition);
        SmartDashboard.putNumber("Elevator/TargetPosition",

                elevatorStateMachine.getTargetProperties().position());
        SmartDashboard.putString("Elevator/State",
                elevatorStateMachine.getCurrentState().toString());
    }

    // Public methods for commanding the elevator
    public void moveElevatorToState(ElevatorState state) {
        if (isElevatorSafe()) {
            elevatorStateMachine.requestTransition(
                state,
                state.properties,
                () -> true, // No check needed for position control
                this::setElevatorPosition);
        } else {
            holdElevatorPosition();
        }
    }

    // assumes that the limit switches will trigger and stop it once it reaches the top or bottom
    public void moveElevatorUp() {
        if (isElevatorSafe()) {
            setPosition(getCurrentPosition() + ElevatorConstants.kElevatorPositionIncrement);
        } else {
            holdElevatorPosition();
        }
        // elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
    }

    // assumes that the limit switches will trigger and stop it once it reaches the top or bottom
    public void moveElevatorDown() {
        if (isElevatorSafe()) {
           setPosition(getCurrentPosition() - ElevatorConstants.kElevatorPositionIncrement);
        } else {
            holdElevatorPosition();
        }
        // elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
    }

    public void stopElevatorSpeed() {
        holdElevatorPosition();
    }

    // Public methods for commanding the wrist

    public ElevatorState getCurrentElevatorState() {
        return elevatorStateMachine.getCurrentState();
    }

    public boolean isElevatorSafe() {

        if (currentPosition <= ElevatorConstants.kElevatorMinHeight || currentPosition >= ElevatorConstants.kElevatorMaxHeight) {
            // holdElevatorPosition();
        }

        if (RobotMap.wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristDangerZoneAngle &&
            (currentPosition > ElevatorConstants.kElevatorDangerZoneStart && currentPosition < ElevatorConstants.kElevatorDangerZoneEnd)) {
            // holdElevatorPosition();
            // return false;
        }
        return true;
    }
}
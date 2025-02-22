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
        elevatorMotor.setBrakeMode(true);
    }

    private void setElevatorPosition(ElevatorProperties properties) {
        double rotations = properties.heightInches() / ElevatorConstants.kInchesPerRotation;
        elevatorMotor.setPosition(rotations);
    }

    @Override
    public void periodic() {
        // Get current positions
        double currentHeight = elevatorMotor.getPosition() * ElevatorConstants.kInchesPerRotation;

        // Update dashboard
        SmartDashboard.putNumber("Elevator/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("Elevator/TargetHeight",
                elevatorStateMachine.getTargetProperties().heightInches());
        SmartDashboard.putString("Elevator/State",
                elevatorStateMachine.getCurrentState().toString());
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

    public ElevatorState getCurrentElevatorState() {
        return elevatorStateMachine.getCurrentState();
    }

}
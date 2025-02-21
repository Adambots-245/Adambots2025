package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
// import com.ctre.phoenix6.signals.ForwardLimitValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.adambots.actuators.BaseMotor;
import com.adambots.actuators.NEOMotor;
import com.adambots.actuators.TalonFXMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.sensors.ThroughBoreEncoder;
import com.adambots.utils.StateMachine;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase {

    // Properties class to hold state data
    public record ElevatorProperties(
            double heightInches, // Height in inches
            double wristAngle,
            String description // Description of state
    ) {
    }

    // Define states with their properties
    public enum ElevatorState {
        INTAKE(new ElevatorProperties(0.0,  20, "Intake Position")),
        L1(new ElevatorProperties(12.0, 40, "Level 1")),
        L2(new ElevatorProperties(24.0, 60, "Level 2")),
        L3(new ElevatorProperties(36.0, 80, "Level 3")),
        L4(new ElevatorProperties(48.0, 100, "Level 4"));

        public final ElevatorProperties properties;

        ElevatorState(ElevatorProperties properties) {
            this.properties = properties;
        }
    }

    // Hardware
    BaseMotor wristMotor;
    BaseMotor elevatorMotor;
    BaseAbsoluteEncoder encoder;

    PIDController pid = new PIDController(0, 0, 0);

    // State Machine
    private final StateMachine<ElevatorState, ElevatorProperties> stateMachine;

    // Constants
    private static final double GEAR_RATIO = 10.0; // 10:1 gear ratio - every 10 rotations of motor is 1 rotation of
    private static final double DRUM_CIRCUMFERENCE = 2.0; // inches - drum or pulley mechanism at the top (Pi*D)
    private static final double INCHES_PER_ROTATION = DRUM_CIRCUMFERENCE / GEAR_RATIO;
    private static final double ElEVATOR_POSITION_TOLERANCE = 0.5; // inches
    private static final double WRIST_POSITION_TOLERANCE = 2; // degrees


    public ElevatorSubsystem(BaseMotor elevatorMotor, BaseMotor wristMotor, BaseAbsoluteEncoder encoder) {
        // Initialize motor
        this.wristMotor = wristMotor;
        this.elevatorMotor = elevatorMotor;
        this.encoder = encoder;

        configureMotor();

        stateMachine = new StateMachine<>(
                ElevatorState.INTAKE,
                ElevatorState.INTAKE.properties,
                message -> SmartDashboard.putString("Elevator/Status", message),
                true // Using position control
        );
    }

    private void configureMotor() {
        elevatorMotor.setPID(0, 0.5, 0.0, 0.0, 0.0);
        elevatorMotor.configureSoftLimits(ElevatorState.L4.properties.heightInches() / INCHES_PER_ROTATION, 0, true);
        elevatorMotor.configureHardLimits(true, false);
        elevatorMotor.setBrakeMode(true);

        wristMotor.setPID(0, 0.5, 0.0, 0.0, 0.0);
        wristMotor.configureSoftLimits(ElevatorState.L4.properties.wristAngle() / INCHES_PER_ROTATION, 0, true);
        wristMotor.configureHardLimits(true, false);
        wristMotor.setBrakeMode(true);
    }

    private void setPosition(ElevatorProperties properties) {
        // Set motor position
        double elevatorRotations = properties.heightInches() / INCHES_PER_ROTATION;
        elevatorMotor.setPosition(elevatorRotations);
    }


    private boolean isWristAtPosition() {
        double currentHeight = (elevatorMotor.getPosition() * INCHES_PER_ROTATION);
        // return Math.abs(currentHeight - stateMachine.getTargetProperties().heightInches()) < POSITION_TOLERANCE;
        double wristSpeed = pid.calculate(encoder.getAbsolutePositionDegrees(), stateMachine.getTargetProperties().wristAngle());
        wristMotor.set(wristSpeed);
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        // Get current position
        double currentHeight = (elevatorMotor.getPosition() * INCHES_PER_ROTATION);
        

        // If the limit switch at the bottom is hit, reset the encoder.
        if (elevatorMotor.getForwardLimitSwitch()) {
            elevatorMotor.setPosition(0);
        }

        // Update dashboard
        SmartDashboard.putNumber("Elevator/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("Elevator/TargetHeight",
                stateMachine.getTargetProperties().heightInches());
        SmartDashboard.putString("Elevator/CurrentState",
                stateMachine.getCurrentState().toString());
        SmartDashboard.putString("Elevator/StateDescription",
                stateMachine.getCurrentState().properties.description());
        SmartDashboard.putBoolean("Elevator/AtPosition", isWristAtPosition());
    }

    // Public methods for commanding the elevator
    public void moveToState(ElevatorState state) {
        stateMachine.requestTransition(
                state,
                state.properties,
                this::isWristAtPosition,
                this::setPosition);
    }
}
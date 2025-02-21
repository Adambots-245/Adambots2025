package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import com.adambots.utils.StateMachine;

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

    // State Machine
    private final StateMachine<ElevatorState, ElevatorProperties> stateMachine;

    // Constants
    private static final double GEAR_RATIO = 10.0; // 10:1 gear ratio - every 10 rotations of motor is 1 rotation of
    private static final double DRUM_CIRCUMFERENCE = 2.0; // inches - drum or pulley mechanism at the top (Pi*D)
    private static final double INCHES_PER_ROTATION = DRUM_CIRCUMFERENCE / GEAR_RATIO;
    private static final double POSITION_TOLERANCE = 0.5; // inches

    public ElevatorSubsystem(BaseMotor elevatorMotor, BaseMotor wristMotor) {
        // Initialize motor
        this.wristMotor = wristMotor;
        this.elevatorMotor = elevatorMotor;

        configureMotor();

        stateMachine = new StateMachine<>(
                ElevatorState.INTAKE,
                ElevatorState.INTAKE.properties,
                message -> SmartDashboard.putString("Elevator/Status", message),
                true // Using position control
        );

        // Put mechanism to dashboard
        // SmartDashboard.putData("Elevator Mechanism", mechanism);
    }

    private void configureMotor() {
        // var config = new TalonFXConfiguration();
        elevatorMotor.setPID(0, 0.5, 0.0, 0.0, 0.0);

        // Configure soft limits
        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorState.L4.properties.heightInches()
        //         / INCHES_PER_ROTATION;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        elevatorMotor.configureSoftLimits(ElevatorState.L4.properties.heightInches() / INCHES_PER_ROTATION, 0, true);

        // Configure forward limit switch (bottom position)
        // config.HardwareLimitSwitch.ForwardLimitEnable = true; // Enable hardware limit
        // config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin; // Normally open
        elevatorMotor.configureHardLimits(true, false);

        // elevatorMotor.getConfigurator().apply(config);
        // elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setBrakeMode(true);
    }

    private void setPosition(ElevatorProperties properties) {
        // Set motor position
        double rotations = properties.heightInches() / INCHES_PER_ROTATION;
        // elevatorMotor.setControl(positionVoltage.withPosition(rotations));
        elevatorMotor.setPosition(rotations);

        // Update visualization
        updateVisualization(properties);
    }

    private void updateVisualization(ElevatorProperties properties) {
        // Calculate carriage position (scale height to visualization)
        double maxHeight = ElevatorState.L4.properties.heightInches();
        double normalizedHeight = properties.heightInches() / maxHeight;
        // double visualHeight = normalizedHeight * 50; // 50 is max visual height

        // Update carriage
        // elevatorCarriage.setLength(5); // Keep constant length
        // elevatorCarriage.setAngle(0); // Keep horizontal
        // elevatorCarriage.setColor(properties.visualColor());

        // // Update position directly on elevatorRoot
        // elevatorRoot.setPosition(30, visualHeight);
    }

    private boolean isAtPosition() {
        // double currentHeight = (elevatorMotor.getPosition().getValueAsDouble() * INCHES_PER_ROTATION);
        double currentHeight = (elevatorMotor.getPosition() * INCHES_PER_ROTATION);
        return Math.abs(currentHeight - stateMachine.getTargetProperties().heightInches()) < POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Get current position
        // double currentHeight = (elevatorMotor.getPosition().getValueAsDouble() * INCHES_PER_ROTATION);
        double currentHeight = (elevatorMotor.getPosition() * INCHES_PER_ROTATION);


        // If the limit switch at the bottom is hit, reset the encoder.
        if (elevatorMotor.getForwardLimitSwitch()) {
            // if (elevatorMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround) {
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
        SmartDashboard.putBoolean("Elevator/AtPosition", isAtPosition());
    }

    // Public methods for commanding the elevator
    public void moveToState(ElevatorState state) {
        stateMachine.requestTransition(
                state,
                state.properties,
                this::isAtPosition,
                this::setPosition);
    }

    // Command factories
    public Command moveToIntakeCommand() {
        return Commands.runOnce(() -> moveToState(ElevatorState.INTAKE));
    }

    public Command moveToL1Command() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L1));
    }

    public Command moveToL2Command() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L2));
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L3));
    }

    public Command moveToL4Command() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L4));
    }
}
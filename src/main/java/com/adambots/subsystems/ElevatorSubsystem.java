package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.adambots.actuators.BaseMotor;
import com.adambots.utils.StateMachine2;

public class ElevatorSubsystem extends SubsystemBase {
    
    // Properties class to hold state data
    public record ElevatorProperties(
        double wristAngleDegrees,      // Angle in degrees
        double elevatorHeight,
        String description       // Description of state

    ) {}

    // Define states with their properties
    public enum ElevatorState {
        INTAKE(new ElevatorProperties(0, 0, "Intake Position")),
        L1(new ElevatorProperties(90.0, 20, "L1 Position")),
        L2(new ElevatorProperties(180.0, 50, "L2 Position")),
        L3(new ElevatorProperties(200, 70, "L3 Position")),
        L4(new ElevatorProperties(230, 90, "L4 Position"));


        public final ElevatorProperties properties;
        
        ElevatorState(ElevatorProperties properties) {
            this.properties = properties;
        }
    }

    // Hardware
    BaseMotor elevatorMotor;
    BaseMotor wristMotor;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    // State Machine
    private final StateMachine2<ElevatorState, ElevatorProperties> stateMachine;

    // Constants
    private static final double GEAR_RATIO = 100.0;
    private static final double MOTOR_ROTATIONS_PER_DEGREE = GEAR_RATIO / 360.0;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double POSITION_TOLERANCE = 5.0; // degrees

    public ElevatorSubsystem(BaseMotor elevatorMotor, BaseMotor wristMotor) {
        // Initialize motor
        armMotor = new TalonFX(1);
        configureMotor();

        // Initialize Mechanism2d
        mechanism = new Mechanism2d(100, 100);
        armPivot = mechanism.getRoot("ArmPivot", 50, 20);
        
        // Create base/tower (vertical gray line)
        armTower = armPivot.append(new MechanismLigament2d(
            "Tower", 20, 90, 4, new Color8Bit(169, 169, 169)));
            
        // Create arm beam (starts horizontal, will rotate)
        armBeam = armPivot.append(new MechanismLigament2d(
            "Arm", 40, 0, 4, new Color8Bit(255, 255, 0)));

        // Initialize state machine
        stateMachine = new StateMachine2<>(
            ElevatorState.INTAKE,
            ElevatorState.INTAKE.properties,
            message -> SmartDashboard.putString("Arm/Status", message),
            true
        );

        // Put mechanism to dashboard
        SmartDashboard.putData("Arm Mechanism", mechanism);
    }

    private void configureMotor() {
        var config = new TalonFXConfiguration();
        
        // Configure PID
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        
        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            ElevatorState.L2.properties.angleDegrees() * MOTOR_ROTATIONS_PER_DEGREE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Configure feedback and control settings
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure status frames for reduced CAN traffic
        // config.StatusFrames.Status_1.setUpdateFrequency(100); // 100ms
        // config.StatusFrames.Status_2.setUpdateFrequency(20);  // 20ms for position feedback

        elevatorMotor.getConfigurator().apply(config);
        armMotor.setPosition(0);
    }

    private void setPosition(ElevatorProperties properties) {
        // Set motor position with optimized control settings
        double rotations = properties.angleDegrees() * MOTOR_ROTATIONS_PER_DEGREE;
        armMotor.setControl(positionVoltage.withPosition(rotations)
                                         .withEnableFOC(false)  // Disable FOC for simpler control
                                         .withSlot(0));         // Explicitly use primary PID slot
    }

    private void updateVisualization(ElevatorProperties properties) {
        // Update arm visualization
        armBeam.setAngle(properties.angleDegrees());
        armBeam.setColor(properties.visualColor());
    }

    private void stopMotor() {
        armMotor.setControl(new DutyCycleOut(0).withEnableFOC(false));
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private boolean isAtPosition() {
        double currentAngle = (armMotor.getPosition().getValueAsDouble() / MOTOR_ROTATIONS_PER_DEGREE);
        return Math.abs(currentAngle - stateMachine.getTargetProperties().angleDegrees()) < POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Get current position
        double currentAngle = (armMotor.getPosition().getValueAsDouble() / MOTOR_ROTATIONS_PER_DEGREE);
        
        // Check if we need to stop
        // if (isAtPosition()) {
        //     stopMotor();
        // }
        
        // Update dashboard
        SmartDashboard.putNumber("Arm/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("Arm/TargetAngle", 
            stateMachine.getTargetProperties().angleDegrees());
        SmartDashboard.putString("Arm/CurrentState", 
            stateMachine.getCurrentState().toString());
        SmartDashboard.putString("Arm/StateDescription", 
            stateMachine.getCurrentState().properties.description());
        SmartDashboard.putBoolean("Arm/AtPosition", isAtPosition());
        
        // Update visualization
        updateVisualization(stateMachine.getTargetProperties());
    }

    // Public methods for commanding the arm
    public void moveToState(ElevatorState state) {
        stateMachine.requestTransition(
            state,
            state.properties,
            this::isAtPosition,
            this::setPosition
        );
    }

    // Command factories
    public Command moveToLowCommand() {
        return Commands.runOnce(() -> moveToState(ElevatorState.INTAKE));
    }

    public Command moveToMidCommand() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L1));
    }

    public Command moveToHighCommand() {
        return Commands.runOnce(() -> moveToState(ElevatorState.L2));
    }
}
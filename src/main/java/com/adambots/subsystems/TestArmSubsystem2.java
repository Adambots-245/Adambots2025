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
import com.adambots.utils.StateMachine2;

public class TestArmSubsystem2 extends SubsystemBase {
    
    // Properties class to hold state data
    public record ArmProperties(
        double angleDegrees,      // Angle in degrees
        Color8Bit visualColor,    // Color for visualization
        String description       // Description of state
    ) {}

    // Define states with their properties
    public enum ArmState {
        LOW(new ArmProperties(0.0, 
            new Color8Bit(0, 255, 0),    // Green
            "Low Position")),
        MID(new ArmProperties(90.0, 
            new Color8Bit(255, 165, 0),  // Orange
            "Mid Position")),
        HIGH(new ArmProperties(180.0, 
            new Color8Bit(255, 0, 0),    // Red
            "High Position"));

        public final ArmProperties properties;
        
        ArmState(ArmProperties properties) {
            this.properties = properties;
        }
    }

    // Hardware
    private final TalonFX armMotor;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    // Mechanism2d visualization
    private final Mechanism2d mechanism;
    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d armTower;
    private final MechanismLigament2d armBeam;

    // State Machine
    private final StateMachine2<ArmState, ArmProperties> stateMachine;

    // Constants
    private static final double GEAR_RATIO = 100.0;
    private static final double MOTOR_ROTATIONS_PER_DEGREE = GEAR_RATIO / 360.0;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double POSITION_TOLERANCE = 5.0; // degrees

    public TestArmSubsystem2() {
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
            ArmState.LOW,
            ArmState.LOW.properties,
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
            ArmState.HIGH.properties.angleDegrees() * MOTOR_ROTATIONS_PER_DEGREE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Configure feedback and control settings
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure status frames for reduced CAN traffic
        // config.StatusFrames.Status_1.setUpdateFrequency(100); // 100ms
        // config.StatusFrames.Status_2.setUpdateFrequency(20);  // 20ms for position feedback

        armMotor.getConfigurator().apply(config);
        armMotor.setPosition(0);
    }

    private void setPosition(ArmProperties properties) {
        // Set motor position with optimized control settings
        double rotations = properties.angleDegrees() * MOTOR_ROTATIONS_PER_DEGREE;
        armMotor.setControl(positionVoltage.withPosition(rotations)
                                         .withEnableFOC(false)  // Disable FOC for simpler control
                                         .withSlot(0));         // Explicitly use primary PID slot
    }

    private void updateVisualization(ArmProperties properties) {
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
    public void moveToState(ArmState state) {
        stateMachine.requestTransition(
            state,
            state.properties,
            this::isAtPosition,
            this::setPosition
        );
    }

    // Command factories
    public Command moveToLowCommand() {
        return Commands.runOnce(() -> moveToState(ArmState.LOW));
    }

    public Command moveToMidCommand() {
        return Commands.runOnce(() -> moveToState(ArmState.MID));
    }

    public Command moveToHighCommand() {
        return Commands.runOnce(() -> moveToState(ArmState.HIGH));
    }
}
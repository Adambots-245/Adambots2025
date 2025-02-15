package com.adambots.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.adambots.utils.StateMachine;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    
    // Simulation state
    private double simRotorPosition = 0.0;
    private double simRotorVelocity = 0.0;
    private double lastTime = 0.0;
    
    // Mechanism2d visualization
    private final Mechanism2d mechanism;
    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d armTower;
    private final MechanismLigament2d armBeam;
    
    // Constants
    private static final double GEAR_RATIO = 100.0;
    private static final double MOTOR_ROTATIONS_PER_DEGREE = GEAR_RATIO / 360.0;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double POSITION_TOLERANCE = 2.0; // degrees
    private static final double SIM_VELOCITY = 9000.0; // degrees per second for simulation
    
    // State Machine context
    public static class ArmContext {
        public double targetDegrees = 0.0;
        public double currentDegrees = 0.0;
        public boolean isMoving = false;
    }
    
    // States
    private final StateMachine<ArmContext> stateMachine;
    private final StateMachine<ArmContext>.State lowState;
    private final StateMachine<ArmContext>.State midState;
    private final StateMachine<ArmContext>.State highState;
    
    public TestArmSubsystem() {
        // Initialize motor
        armMotor = new TalonFX(1);
        configureMotor();
        
        // Initialize Mechanism2d
        mechanism = new Mechanism2d(100, 100);
        armPivot = mechanism.getRoot("ArmPivot", 50, 20);
        armTower = armPivot.append(new MechanismLigament2d(
            "Tower", 20, 90, 4, new Color8Bit(169, 169, 169)));
        armBeam = armPivot.append(new MechanismLigament2d(
            "Arm", 40, 0, 4, new Color8Bit(255, 255, 0)));
        
        // Publish mechanism to dashboard
        SmartDashboard.putData("Arm Mechanism", mechanism);
        
        // Initialize State Machine
        ArmContext context = new ArmContext();
        stateMachine = new StateMachine<>("TestArm", context, this::log);
        
        // Create states
        lowState = stateMachine.addState("Low", () -> 
            Math.abs(context.currentDegrees) < POSITION_TOLERANCE);
            
        midState = stateMachine.addState("Mid", () -> 
            Math.abs(context.currentDegrees - 90.0) < POSITION_TOLERANCE);
            
        highState = stateMachine.addState("High", () -> 
            Math.abs(context.currentDegrees - 180.0) < POSITION_TOLERANCE);
        
        // Add transitions
        lowState.addTransition(midState, 
            ctx -> {
                log("Moving to mid position");
                ctx.targetDegrees = 90.0;
                ctx.isMoving = true;
                setTargetPosition(90.0);
            },
            ctx -> {
                log("Reached mid position");
                ctx.isMoving = false;
                stopMotor();
            }
        );
        
        lowState.addTransition(highState, 
            ctx -> {
                log("Moving to high position");
                ctx.targetDegrees = 180.0;
                ctx.isMoving = true;
                setTargetPosition(180.0);
            },
            ctx -> {
                log("Reached high position");
                ctx.isMoving = false;
                stopMotor();
            }
        );
        
        midState.addTransition(lowState, 
            ctx -> {
                log("Moving to low position");
                ctx.targetDegrees = 0.0;
                ctx.isMoving = true;
                setTargetPosition(0.0);
            },
            ctx -> {
                log("Reached low position");
                ctx.isMoving = false;
                stopMotor();
            }
        );
        
        midState.addTransition(highState, 
            ctx -> {
                log("Moving to high position");
                ctx.targetDegrees = 180.0;
                ctx.isMoving = true;
                setTargetPosition(180.0);
            },
            ctx -> {
                log("Reached high position");
                ctx.isMoving = false;
                stopMotor();
            }
        );
        
        highState.addTransition(lowState, 
            ctx -> {
                log("Moving to low position");
                ctx.targetDegrees = 0.0;
                ctx.isMoving = true;
                setTargetPosition(0.0);
            },
            ctx -> {
                log("Reached low position");
                ctx.isMoving = false;
                stopMotor();
            }
        );
        
        highState.addTransition(midState, 
            ctx -> {
                log("Moving to mid position");
                ctx.targetDegrees = 90.0;
                ctx.isMoving = true;
                setTargetPosition(90.0);
            },
            ctx -> {
                log("Reached mid position");
                ctx.isMoving = false;
                stopMotor();
            }
        );

        lastTime = Timer.getFPGATimestamp();
    }

    private void log(String message) {
        System.out.println("[ArmSubsystem] " + message);
        SmartDashboard.putString("ArmSubsystem/Status", message);
        DataLogManager.log("[ArmSubsystem] " + message);
    }
    
    private void configureMotor() {
        var config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 100;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        armMotor.getConfigurator().apply(config);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.setPosition(0);
    }
    
    private void setTargetPosition(double degrees) {
        if (RobotBase.isReal()) {
            double rotations = degrees * MOTOR_ROTATIONS_PER_DEGREE;
            armMotor.setControl(positionVoltage.withPosition(rotations));
        } else {
            double currentDegrees = simRotorPosition / MOTOR_ROTATIONS_PER_DEGREE;
            double error = degrees - currentDegrees;
            
            // Full speed until very close to target
            simRotorVelocity = Math.signum(error) * (SIM_VELOCITY / 360.0);
            if (Math.abs(error) < 2) {  // Only slow down in final 2 degrees
                simRotorVelocity *= 0.25;
            }
        }
    }
    
    private void stopMotor() {
        if (RobotBase.isReal()) {
            armMotor.setControl(new DutyCycleOut(0));
            armMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            simRotorVelocity = 0;
        }
        log("Motor stopped");
    }
    
    @Override
    public void periodic() {
        // Update simulation
        if (!RobotBase.isReal()) {
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastTime;
            
            // Update position based on velocity
            simRotorPosition += simRotorVelocity * deltaTime;
            
            // Get current position in degrees
            double currentDegrees = simRotorPosition / MOTOR_ROTATIONS_PER_DEGREE;
            
            // Update context
            stateMachine.getContext().currentDegrees = currentDegrees;
            
            // Update last time
            lastTime = currentTime;

            // Debug output
            SmartDashboard.putNumber("Debug/CurrentDegrees", currentDegrees);
            SmartDashboard.putNumber("Debug/TargetDegrees", stateMachine.getContext().targetDegrees);
            SmartDashboard.putNumber("Debug/Velocity", simRotorVelocity);
            SmartDashboard.putBoolean("Debug/IsMoving", stateMachine.getContext().isMoving);
        } else {
            // Real robot code
            double currentRotations = armMotor.getRotorPosition().getValueAsDouble();
            stateMachine.getContext().currentDegrees = currentRotations / MOTOR_ROTATIONS_PER_DEGREE;
        }
        
        // Update state machine
        stateMachine.periodic();
        
        // Update visualization
        armBeam.setAngle(stateMachine.getContext().currentDegrees);
        
        // Update dashboard
        SmartDashboard.putNumber("Arm/CurrentDegrees", stateMachine.getContext().currentDegrees);
        SmartDashboard.putNumber("Arm/TargetDegrees", stateMachine.getContext().targetDegrees);
        SmartDashboard.putString("Arm/CurrentState", stateMachine.getCurrentState().getName());
    }
    
    // Public control methods
    public void moveToLow() {
        stateMachine.requestTransition(lowState);
    }
    
    public void moveToMid() {
        stateMachine.requestTransition(midState);
    }
    
    public void moveToHigh() {
        stateMachine.requestTransition(highState);
    }

    public Command moveToLowCommand() {
        return Commands.runOnce(this::moveToLow);
    }

    public Command moveToMidCommand() {
        return Commands.runOnce(this::moveToMid);
    }

    public Command moveToHighCommand() {
        return Commands.runOnce(this::moveToHigh);
    }
}
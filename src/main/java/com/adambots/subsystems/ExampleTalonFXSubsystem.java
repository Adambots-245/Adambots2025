package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor.ControlMode;
import com.adambots.actuators.TalonFXMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Example subsystem that uses a TalonFX motor with simulation support.
 * This example shows how to properly initialize and update the simulation.
 */
public class ExampleTalonFXSubsystem extends SubsystemBase {
    // Motor constants
    private static final int MOTOR_PORT = 1;
    private static final boolean IS_ON_CANIVORE = false;
    private static final double CURRENT_LIMIT = 40.0;
    private static final boolean IS_KRAKEN = false;
    
    // PID Constants
    private static final double kP = 0.05;
    private static final double kI = 0.0001;
    private static final double kD = 0.1;
    private static final double kF = 0.045;
    
    // Motion Magic Constants
    private static final double CRUISE_VELOCITY = 5.0; // rotations per second
    private static final double ACCELERATION = 10.0; // rotations per second squared
    private static final double JERK = 50.0; // rotations per second cubed
    
    // Simulation Constants
    private static final double MOMENT_OF_INERTIA = 0.005; // kg*m^2
    private static final double GEAR_RATIO = 10.0; // 10:1 reduction
    
    // The motor controller
    private final TalonFXMotor motor;
    
    // Target position for this subsystem
    private double targetPosition = 0.0;
    
    /**
     * Creates a new ExampleTalonFXSubsystem.
     */
    public ExampleTalonFXSubsystem() {
        // Initialize the motor
        motor = new TalonFXMotor(MOTOR_PORT, IS_ON_CANIVORE, CURRENT_LIMIT, IS_KRAKEN);
        
        // Configure the motor
        configureMotor();
        
        // Additional simulation-specific configuration
        if (RobotBase.isSimulation()) {
            // Set custom simulation parameters (moment of inertia, gear ratio)
            motor.setSimulationParameters(MOMENT_OF_INERTIA, GEAR_RATIO);
            
            // Initialize simulation position
            motor.setSimulationPosition(0.0);
        }
    }
    
    /**
     * Configures the motor with PID values, motion magic, and other settings.
     */
    private void configureMotor() {
        // Set brake mode
        motor.setBrakeMode(true);
        
        // Configure PID for slot 0
        motor.setPID(0, kP, kI, kD, kF);
        
        // Configure Motion Magic
        motor.configureMotionMagic(CRUISE_VELOCITY, ACCELERATION, JERK);
        
        // Configure soft limits
        motor.configureSoftLimits(10.0, -10.0, true);
        
        // Configure current limits
        motor.configureCurrentLimits(40.0, 30.0, 0.0);
        
        // Enable voltage compensation
        motor.enableVoltageCompensation(12.0);
    }
    
    /**
     * Sets the target position for the subsystem.
     * 
     * @param position Target position in rotations
     */
    public void setTargetPosition(double position) {
        targetPosition = position;
        
        // Use Motion Magic for smoother position control
        motor.set(ControlMode.MOTION_MAGIC, targetPosition);
    }
    
    /**
     * Gets the current position of the subsystem.
     * 
     * @return Current position in rotations
     */
    public double getCurrentPosition() {
        return motor.getPosition();
    }
    
    /**
     * Sets the motor to run at the specified percentage of maximum output.
     * 
     * @param percentOutput The percentage output (-1.0 to 1.0)
     */
    public void setPercentOutput(double percentOutput) {
        motor.set(ControlMode.PERCENT_OUTPUT, percentOutput);
    }
    
    /**
     * Periodic method called by WPILib's scheduler.
     * This method is called approximately every 20ms.
     */
    @Override
    public void periodic() {
        // Update SmartDashboard with motor data
        SmartDashboard.putNumber("Motor Position", motor.getPosition());
        SmartDashboard.putNumber("Motor Velocity", motor.getVelocity());
        SmartDashboard.putNumber("Motor Current", motor.getCurrentDraw());
        SmartDashboard.putNumber("Motor Temperature", motor.getTemperature());
        SmartDashboard.putBoolean("Forward Limit", motor.getForwardLimitSwitch());
        SmartDashboard.putBoolean("Reverse Limit", motor.getReverseLimitSwitch());
        SmartDashboard.putNumber("Target Position", targetPosition);
    }
    
    /**
     * Simulation periodic method called by WPILib's scheduler during simulation.
     * This method is called after the periodic() method.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation with the current timestamp
        // Using 0.02 seconds as the default period (20ms)
        motor.simulationPeriodic(0.02);
        
        // Simulate battery voltage drop based on current draw
        double currentDraw = motor.getCurrentDraw();
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(currentDraw)
        );
        
        // Additional simulation code as needed
        SmartDashboard.putNumber("Sim Current Draw", currentDraw);
        SmartDashboard.putNumber("Sim Battery Voltage", RobotController.getBatteryVoltage());
    }
    
    /**
     * Resets the simulation state.
     * This method can be called during simulation to reset the subsystem.
     */
    public void resetSimulation() {
        if (RobotBase.isSimulation()) {
            motor.resetSimulation();
            targetPosition = 0.0;
        }
    }
    
    /**
     * Example method to demonstrate hitting limit switches in simulation.
     * This could be bound to buttons in a command-based robot.
     */
    public void simulateLimitSwitchHit(boolean isForward) {
        if (RobotBase.isSimulation()) {
            motor.setSimulatedLimitSwitch(isForward, true);
        }
    }
    
    /**
     * Example method to clear simulated limit switches.
     */
    public void clearSimulatedLimitSwitches() {
        if (RobotBase.isSimulation()) {
            motor.setSimulatedLimitSwitch(true, false);
            motor.setSimulatedLimitSwitch(false, false);
        }
    }
}
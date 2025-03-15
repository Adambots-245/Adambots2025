package com.adambots.actuators;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;

/**
 * A wrapper class for the CTRE TalonFX motor controller that implements the BaseMotor interface.
 * This class provides a standardized interface for controlling TalonFX motors, including
 * Falcon 500 and Kraken X60 motors, with support for various control modes, configuration options,
 * status reporting, and simulation functionality.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Multiple control modes (Percent Output, Position, Velocity, etc.)</li>
 *   <li>PID configuration with multiple slots</li>
 *   <li>Motion Magic configuration</li>
 *   <li>Current limiting</li>
 *   <li>Soft and hard limit switch support</li>
 *   <li>Voltage compensation</li>
 *   <li>Motor following capabilities</li>
 *   <li>Simulation support for WPILib Simulation mode</li>
 * </ul>
 * 
 * <p>The class automatically optimizes CAN bus usage by configuring appropriate
 * update frequencies for different status signals and includes special handling
 * for Kraken X60 motors when using current control mode.
 * 
 * <p>Example usage:
 * <pre>
 * TalonFXMotor motor = new TalonFXMotor(1, true, 40, false);
 * motor.setBrakeMode(true);
 * motor.set(ControlMode.PERCENT_OUTPUT, 0.5);
 * </pre>
 * 
 * @see BaseMotor
 * @see com.ctre.phoenix6.hardware.TalonFX
 */
public class TalonFXMotor implements BaseMotor {
    private final TalonFX motor;
    private final boolean isKraken;
    private final int motorId;

    // Simulation-specific fields
    private final TalonFXSimState simState;
    private FlywheelSim motorSim;
    private double simPosition = 0;
    private double simVelocity = 0;
    private double simAcceleration = 0;
    private double simTemperature = 25.0; // Default temperature in Celsius
    private double simCurrent = 0;
    private double simVoltage = 12.0; // Default voltage
    private double lastSimTime = 0;
    private double lastSimVelocity = 0;
    private boolean forwardLimitTriggered = false;
    private boolean reverseLimitTriggered = false;
    private double forwardSoftLimit = Double.POSITIVE_INFINITY;
    private double reverseSoftLimit = Double.NEGATIVE_INFINITY;
    private boolean softLimitsEnabled = false;
    private double targetPosition = 0;
    private double targetVelocity = 0;
    private boolean brakeMode = false;
    
    // PID simulation parameters
    private double simKp = 0;
    private double simKi = 0;
    private double simKd = 0;
    private double simKf = 0;
    private int activePidSlot = 0;
    private ControlMode currentControlMode = ControlMode.PERCENT_OUTPUT;
    private double currentSetpoint = 0;
    private double integralAccumulator = 0;
    private double lastError = 0;
    
    // Motion Magic parameters
    private double cruiseVelocity = 0;
    private double acceleration = 0;
    private double jerk = 0;
    private boolean motionMagicActive = false;
    
    // Gear ratio for simulation
    private double gearRatio = 1.0;

    /**
     * Constructs a TalonFXMotor instance.
     *
     * @param portNum The port number to which the motor is connected.
     * @param isOnCANivore A boolean indicating if the motor is on a CANivore bus.
     * @param supplyCurrentLimit The supply current limit for the motor.
     * @param isKraken A boolean indicating if the motor is part of the Kraken subsystem.
     */
    public TalonFXMotor(int portNum, boolean isOnCANivore, double supplyCurrentLimit, boolean isKraken) {
        this.isKraken = isKraken;
        this.motorId = portNum;

        // Initialize motor on either CANivore or regular CAN bus
        if (isOnCANivore) {
            motor = new TalonFX(portNum, "*");
        } else {
            motor = new TalonFX(portNum);
        }

        // Configure default current limits
        var currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(supplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        motor.getConfigurator().apply(currentLimits);

        // Configure status frame periods for efficiency
        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(50);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage
        motor.optimizeBusUtilization();
        
        // Initialize simulation components
        simState = motor.getSimState();
        
        // Set default orientation for simulation
        if (RobotBase.isSimulation()) {
            simState.Orientation = ChassisReference.CounterClockwise_Positive;
        }
        
        // Initialize FlywheelSim with appropriate motor model
        // Default to Falcon 500 parameters unless specified as Kraken
        if (isKraken) {
            // Kraken X60 motor model
            DCMotor motor = DCMotor.getKrakenX60(1);
            motorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    motor.KvRadPerSecPerVolt,
                    motor.KtNMPerAmp / motor.KvRadPerSecPerVolt),
                motor,
                1.0  // Gear ratio (1:1 by default)
            );
        } else {
            // Falcon 500 motor model
            DCMotor motor = DCMotor.getFalcon500(1);
            motorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    motor.KvRadPerSecPerVolt,
                    motor.KtNMPerAmp / motor.KvRadPerSecPerVolt),
                motor,
                1.0  // Gear ratio (1:1 by default)
            );
        }
    }

    /**
     * Sets the control mode and value for the motor.
     * 
     * @param mode The ControlMode to set the motor to (PERCENT_OUTPUT, POSITION, VELOCITY, 
     *             VOLTAGE, CURRENT, MOTION_MAGIC, or FOLLOWER)
     * @param value The target value to set:
     *              - For PERCENT_OUTPUT: Output value between -1.0 and 1.0
     *              - For POSITION: Target position in sensor units
     *              - For VELOCITY: Target velocity in sensor units per second
     *              - For VOLTAGE: Target voltage
     *              - For CURRENT: Target current in amps
     *              - For MOTION_MAGIC: Target position in sensor units
     *              - For FOLLOWER: Device ID of the motor to follow
     */
    @Override
    public void set(ControlMode mode, double value) {
        // Store control mode and setpoint for simulation
        currentControlMode = mode;
        currentSetpoint = value;
        
        // Real hardware control
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(new DutyCycleOut(value));
                break;
            case POSITION:
                motor.setControl(new PositionDutyCycle(value).withSlot(activePidSlot));
                break;
            case VELOCITY:
                motor.setControl(new VelocityDutyCycle(value).withSlot(activePidSlot));
                break;
            case VOLTAGE:
                motor.setControl(new VoltageOut(value));
                break;
            case CURRENT:
                if (isKraken) {
                    // Kraken X60 supports FOC (Field Oriented Control) for better current control
                    motor.setControl(new TorqueCurrentFOC(value));
                } else {
                    // For standard TalonFX, use voltage control mode as an approximation
                    // since non-FOC current control is not directly available in Phoenix 6
                    double percentOutput = value / 40.0; // Convert amps to approximate percent output
                    percentOutput = MathUtil.clamp(percentOutput, -1.0, 1.0);
                    motor.setControl(new DutyCycleOut(percentOutput));
                }
                break;
            case MOTION_MAGIC:
                motor.setControl(new MotionMagicDutyCycle(value).withSlot(activePidSlot));
                break;
            case FOLLOWER:
                // Follow another Talon FX controller
                int deviceID = (int) value;
                motor.setControl(new Follower(deviceID, false));
                break;
        }
    }

    /**
     * Sets the speed of the TalonFX motor using duty cycle output control.
     * 
     * @param speed The speed to set the motor to, ranging from -1.0 to 1.0.
     *             Positive values indicate forward rotation, negative values indicate reverse rotation,
     *             and 0.0 represents stopped.
     */
    @Override
    public void set(double speed) {
        set(ControlMode.PERCENT_OUTPUT, speed);
    }

    /**
     * Sets the PID (Proportional, Integral, Derivative) and Feed Forward control constants for the motor controller.
     * The motor controller supports multiple PID slot configurations (0-2) that can be used for different control modes.
     *
     * @param slotIdx The PID slot index to configure (0-2)
     * @param kP The Proportional gain constant
     * @param kI The Integral gain constant  
     * @param kD The Derivative gain constant
     * @param kF The Feed Forward gain constant
     * @throws IllegalArgumentException if slotIdx is not between 0-2
     */
    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        // Store PID values for simulation
        if (slotIdx == 0) {
            simKp = kP;
            simKi = kI;
            simKd = kD;
            simKf = kF;
        }
        
        // Apply PID configuration to the motor
        switch (slotIdx) {
            case 0:
                var slot0Config = new Slot0Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot0Config);
                break;

            case 1:
                var slot1Config = new Slot1Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot1Config);
                break;

            case 2:
                var slot2Config = new Slot2Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot2Config);
                break;

            default:
                throw new IllegalArgumentException("Invalid slot index. Must be between 0 and 3.");
        }
    }

    /**
     * Configures motion magic parameters for the TalonFX motor.
     * Motion Magic is a control mode that provides smooth position control using a trapezoidal motion profile.
     *
     * @param cruiseVelocity The cruise velocity in sensor units per second
     * @param acceleration The acceleration in sensor units per second per second
     * @param jerk The jerk (rate of acceleration change) in sensor units per second per second per second
     */
    @Override
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        // Store Motion Magic parameters for simulation
        this.cruiseVelocity = cruiseVelocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        
        // Apply Motion Magic configuration to the motor
        var config = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicJerk(jerk);
        motor.getConfigurator().apply(config);
    }

    /**
     * Configures current limits for the TalonFX motor.
     * 
     * @param stallLimit The stator current limit (in amperes) when motor is stalled
     * @param freeLimit The supply current limit (in amperes) when motor is running freely
     * @param limitRPM The RPM threshold for current limiting (not used in current implementation)
     */
    @Override
    public void configureCurrentLimits(double stallLimit, double freeLimit, double limitRPM) {
        var config = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(stallLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(freeLimit)
                .withSupplyCurrentLimitEnable(true);
        motor.getConfigurator().apply(config);
    }

    /**
     * Configures software limits for the TalonFX motor.
     * Software limits prevent the motor from moving beyond specified forward and reverse positions.
     *
     * @param forwardLimit The maximum forward position the motor can move to
     * @param reverseLimit The minimum reverse position the motor can move to
     * @param enable Boolean flag to enable/disable both forward and reverse soft limits
     */
    @Override
    public void configureSoftLimits(double forwardLimit, double reverseLimit, boolean enable) {
        // Store soft limits for simulation
        forwardSoftLimit = forwardLimit;
        reverseSoftLimit = reverseLimit;
        softLimitsEnabled = enable;
        
        // Apply soft limit configuration to the motor
        var config = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(forwardLimit)
                .withReverseSoftLimitThreshold(reverseLimit)
                .withForwardSoftLimitEnable(enable)
                .withReverseSoftLimitEnable(enable);
        motor.getConfigurator().apply(config);
    }

    /**
     * Enables or disables software limit switches for both forward and reverse directions.
     * Software limits prevent the motor from moving beyond specified positions.
     * 
     * @param enable true to enable software limits, false to disable them
     */
    @Override
    public void enableSoftLimits(boolean enable) {
        softLimitsEnabled = enable;
        
        var config = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(enable)
                .withReverseSoftLimitEnable(enable);
        motor.getConfigurator().apply(config);
    }

    /**
     * Sets the inversion state of the motor.
     * 
     * @param inverted true to invert the motor (Clockwise is positive), 
     *                 false for normal operation (CounterClockwise is positive)
     */
    @Override
    public void setInverted(boolean inverted) {
        var config = new MotorOutputConfigs()
                .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        motor.getConfigurator().apply(config);
    }

    /**
     * Sets the neutral mode of the motor to either brake or coast.
     * In brake mode, the motor actively resists motion when not driven.
     * In coast mode, the motor spins freely when not driven.
     *
     * @param brake true to enable brake mode, false for coast mode
     */
    @Override
    public void setBrakeMode(boolean brake) {
        brakeMode = brake;
        
        var config = new MotorOutputConfigs()
                .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        motor.getConfigurator().apply(config);
    }

    /**
     * Sets the position of the motor in rotations.
     * 
     * @param rotations The target position in rotations for the motor to move to
     */
    @Override
    public void setPosition(double rotations) {
        set(ControlMode.POSITION, rotations);
    }

    /**
     * Enables voltage compensation for the motor.
     * This helps maintain consistent motor performance regardless of battery voltage fluctuations.
     *
     * @param voltage The voltage to compensate to (in volts). 
     *               The motor will scale its output to maintain consistent behavior at this voltage level.
     */
    @Override
    public void enableVoltageCompensation(double voltage) {
        simVoltage = voltage;
        
        var config = new VoltageConfigs()
                .withPeakForwardVoltage(voltage)
                .withPeakReverseVoltage(-voltage);
        motor.getConfigurator().apply(config);
    }

    /**
     * Gets the current position of the motor.
     * 
     * @return The current position of the motor in rotations (double)
     */
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current velocity of the motor.
     *
     * @return The current velocity in units per second
     */
    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current acceleration of the motor.
     * @return The current acceleration in units per second^2
     */
    @Override
    public double getAcceleration() {
        return motor.getAcceleration().getValueAsDouble();
    }

    /**
     * Gets the current draw of the motor.
     * @return The current draw in amps
     */
    @Override
    public double getCurrentDraw() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Gets the current output percentage of the motor.
     * @return The current output percentage as a double between -1.0 and 1.0
     */
    @Override
    public double getOutputPercent() {
        return motor.getDutyCycle().getValueAsDouble();
    }

    /**
     * Gets the current temperature of the motor.
     * @return The current temperature in degrees Celsius
     */
    @Override
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the state of the forward limit switch.
     * @return True if the forward limit switch is closed, false otherwise
     */
    @Override
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets the state of the reverse limit switch.
     * @return True if the reverse limit switch is closed, false otherwise
     */
    @Override
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Sets this motor as a strict follower of another motor controller.
     * @param deviceID The ID of the motor controller to follow
     */
    @Override
    public void setStrictFollower(int deviceID) {
        // Set this motor as a follower using the Follower control request
        set(ControlMode.FOLLOWER, deviceID);
    }

    /**
     * Configures hard limits for the TalonFX motor.
     * @param enableForward True to enable the forward hard limit, false otherwise
     * @param enableReverse True to enable the reverse hard limit, false otherwise
     * @param forwardValue Position value to set when forward limit is triggered
     * @param reverseValue Position value to set when reverse limit is triggered
     */
    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse, double forwardValue, double reverseValue) {
        var limitSwitchConfigs = new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(enableForward)
                .withForwardLimitAutosetPositionEnable(enableForward)
                .withForwardLimitAutosetPositionValue(forwardValue)
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                .withReverseLimitEnable(enableReverse)
                .withReverseLimitAutosetPositionEnable(enableReverse)
                .withReverseLimitAutosetPositionValue(reverseValue)
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);

        motor.getConfigurator().apply(limitSwitchConfigs);

        // For simulation/feedback
        motor.getForwardLimit().setUpdateFrequency(50);
        motor.getReverseLimit().setUpdateFrequency(50);

        // Set up simulation state
        simState.setForwardLimit(enableForward);
        simState.setReverseLimit(enableReverse);
    }
    
    /**
     * Updates the simulation model with the latest control values.
     * This method should be called periodically (typically in the periodic method of a subsystem)
     * to update the simulation state of the motor.
     * 
     * @param dtSeconds The delta time in seconds since the last update
     */
    public void simulationPeriodic(double dtSeconds) {
        if (!RobotBase.isSimulation()) {
            return; // Only run in simulation mode
        }
        
        // Set the supply voltage for simulation
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Calculate motor output based on current control mode
        double motorOutput = 0;
        
        switch (currentControlMode) {
            case PERCENT_OUTPUT:
                motorOutput = currentSetpoint;
                break;
                
            case POSITION:
                // Simple PID position control simulation
                double positionError = targetPosition - simPosition;
                integralAccumulator += positionError * dtSeconds;
                double derivative = (positionError - lastError) / dtSeconds;
                
                // PID calculation
                motorOutput = (simKp * positionError) + 
                             (simKi * integralAccumulator) + 
                             (simKd * derivative);
                
                // Apply feed-forward if applicable
                motorOutput += simKf * Math.signum(positionError);
                
                // Clamp output to valid range
                motorOutput = MathUtil.clamp(motorOutput, -1.0, 1.0);
                
                lastError = positionError;
                break;
                
            case VELOCITY:
                // Simple PID velocity control simulation
                double velocityError = targetVelocity - simVelocity;
                integralAccumulator += velocityError * dtSeconds;
                double velocityDerivative = (velocityError - lastError) / dtSeconds;
                
                // PID calculation
                motorOutput = (simKp * velocityError) + 
                             (simKi * integralAccumulator) + 
                             (simKd * velocityDerivative) + 
                             (simKf * targetVelocity); // Feed-forward based on target velocity
                
                // Clamp output to valid range
                motorOutput = MathUtil.clamp(motorOutput, -1.0, 1.0);
                
                lastError = velocityError;
                break;
                
            case MOTION_MAGIC:
                if (motionMagicActive) {
                    // Simple motion profiling simulation
                    double posError = targetPosition - simPosition;
                    double distanceToTarget = Math.abs(posError);
                    
                    // Calculate deceleration distance based on current velocity and acceleration
                    double decelDistance = (simVelocity * simVelocity) / (2.0 * acceleration);
                    
                    // Determine if we need to accelerate, cruise, or decelerate
                    if (distanceToTarget > decelDistance) {
                        // Accelerate or cruise
                        if (Math.abs(simVelocity) < cruiseVelocity) {
                            // Accelerate
                            double targetSpeed = Math.min(cruiseVelocity, 
                                Math.abs(simVelocity) + acceleration * dtSeconds);
                            targetSpeed *= Math.signum(posError);
                            targetVelocity = targetSpeed;
                        } else {
                            // Cruise at max velocity
                            targetVelocity = cruiseVelocity * Math.signum(posError);
                        }
                    } else {
                        // Decelerate
                        double targetSpeed = Math.max(0.05, 
                            Math.sqrt(2.0 * acceleration * distanceToTarget));
                        targetSpeed *= Math.signum(posError);
                        targetVelocity = targetSpeed;
                    }
                    
                    // Use velocity PID to achieve the target velocity
                    double velError = targetVelocity - simVelocity;
                    motorOutput = simKp * velError + simKf * targetVelocity;
                    motorOutput = MathUtil.clamp(motorOutput, -1.0, 1.0);
                    
                    // If we've reached the target, disable motion magic
                    if (Math.abs(posError) < 0.001 && Math.abs(simVelocity) < 0.001) {
                        motionMagicActive = false;
                    }
                }
                break;
                
            case VOLTAGE:
                // For voltage control, convert to percent output (assuming 12V nominal)
                motorOutput = currentSetpoint / 12.0;
                break;
                
            case CURRENT:
                // Current control is simplified in simulation
                motorOutput = currentSetpoint / 40.0; // Approximation assuming 40A is max current
                break;
                
            case FOLLOWER:
                // No simple way to simulate follower mode without reference to the other motor
                motorOutput = 0;
                break;
        }
        
        // Apply motor output to simulation
        double appliedVoltage = motorOutput * 12.0; // Convert from -1 to 1 to voltage
        
        // Check if we're hitting a limit
        if (softLimitsEnabled) {
            if ((simPosition >= forwardSoftLimit && appliedVoltage > 0) || 
                (simPosition <= reverseSoftLimit && appliedVoltage < 0)) {
                appliedVoltage = 0;
            }
        }
        
        // Update hardware limit switches based on position
        if (simPosition >= forwardSoftLimit) {
            forwardLimitTriggered = true;
            simState.setForwardLimit(true);
            if (appliedVoltage > 0) {
                appliedVoltage = 0; // Stop motor if trying to go beyond limit
            }
        } else {
            forwardLimitTriggered = false;
            simState.setForwardLimit(false);
        }
        
        if (simPosition <= reverseSoftLimit) {
            reverseLimitTriggered = true;
            simState.setReverseLimit(true);
            if (appliedVoltage < 0) {
                appliedVoltage = 0; // Stop motor if trying to go beyond limit
            }
        } else {
            reverseLimitTriggered = false;
            simState.setReverseLimit(false);
        }
        
        // Use the FlywheelSim to simulate the physics
        motorSim.setInputVoltage(appliedVoltage);
        motorSim.update(dtSeconds);
        
        // Get the updated state from the simulation
        simVelocity = motorSim.getAngularVelocityRPM() / 60.0; // Convert RPM to RPS
        
        // Update position based on velocity
        simPosition += simVelocity * dtSeconds;
        
        // Calculate acceleration
        simAcceleration = (simVelocity - lastSimVelocity) / dtSeconds;
        lastSimVelocity = simVelocity;
        
        // Update current draw simulation
        simCurrent = motorSim.getCurrentDrawAmps();
        
        // Simulate temperature rise based on current (simple model)
        // Temperature rises with current and falls when idle
        double tempRiseRate = 0.01 * simCurrent * simCurrent; // Quadratic relationship with current
        double tempFallRate = 0.05 * (25.0 - simTemperature); // Linear cooling to ambient (25°C)
        simTemperature += (tempRiseRate + tempFallRate) * dtSeconds;
        
        // Update TalonFX simulation state
        // Note: In Phoenix 6, we use setRawRotorPosition and setRotorVelocity
        simState.setRawRotorPosition(simPosition);
        simState.setRotorVelocity(simVelocity);
        simState.setRotorAcceleration(simAcceleration);
    }
    
    /**
     * Resets the simulation state to initial values.
     * Useful for testing and resetting between simulation runs.
     */
    public void resetSimulation() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        simPosition = 0;
        simVelocity = 0;
        simAcceleration = 0;
        simTemperature = 25.0;
        simCurrent = 0;
        lastSimVelocity = 0;
        forwardLimitTriggered = false;
        reverseLimitTriggered = false;
        integralAccumulator = 0;
        lastError = 0;
        motionMagicActive = false;
        
        simState.setRawRotorPosition(0);
        simState.setRotorVelocity(0);
        simState.setRotorAcceleration(0);
        simState.setForwardLimit(false);
        simState.setReverseLimit(false);
    }
    
    /**
     * Sets custom simulation parameters for the motor model.
     * This allows for customization of the simulation physics.
     * 
     * @param momentOfInertia The moment of inertia for the motor load (kg*m^2)
     * @param gearRatio The gear ratio between the motor and output shaft
     */
    public void setSimulationParameters(double momentOfInertia, double gearRatio) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        this.gearRatio = gearRatio;
        
        // Re-create the simulation model with custom parameters
        if (isKraken) {
            DCMotor motor = DCMotor.getKrakenX60(1);
            motorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    motor.KvRadPerSecPerVolt * gearRatio,
                    motor.KtNMPerAmp / motor.KvRadPerSecPerVolt),
                motor,
                gearRatio
            );
        } else {
            DCMotor motor = DCMotor.getFalcon500(1);
            motorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    motor.KvRadPerSecPerVolt * gearRatio,
                    motor.KtNMPerAmp / motor.KvRadPerSecPerVolt),
                motor,
                gearRatio
            );
        }
    }
    
    /**
     * Manually sets the simulated position of the motor.
     * Useful for testing specific positions without driving the motor.
     * 
     * @param position The position to set in rotations
     */
    public void setSimulationPosition(double position) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        simPosition = position;
        simState.setRawRotorPosition(position);
    }
    
    /**
     * Manually sets the simulated velocity of the motor.
     * Useful for testing specific velocities without driving the motor.
     * 
     * @param velocity The velocity to set in rotations per second
     */
    public void setSimulationVelocity(double velocity) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        simVelocity = velocity;
        simState.setRotorVelocity(velocity);
    }
    
    /**
     * Triggers a simulated hardware limit switch.
     * This can be used to test limit switch behavior.
     * 
     * @param forward True to trigger the forward limit, false for reverse
     * @param triggered True to set the limit as triggered, false to clear
     */
    public void setSimulatedLimitSwitch(boolean forward, boolean triggered) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        if (forward) {
            forwardLimitTriggered = triggered;
            simState.setForwardLimit(triggered);
        } else {
            reverseLimitTriggered = triggered;
            simState.setReverseLimit(triggered);
        }
    }
}
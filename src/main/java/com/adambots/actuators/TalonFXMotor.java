package com.adambots.actuators;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

/**
 * A wrapper class for the CTRE TalonFX motor controller that implements the BaseMotor interface.
 * This class provides a standardized interface for controlling TalonFX motors, including
 * Falcon 500 and Kraken X60 motors, with support for various control modes, configuration options,
 * and status reporting.
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
    }

    /**
     * Sets the control mode and value for the motor.
     * 
     * @param mode The ControlMode to set the motor to (PERCENT_OUTPUT, POSITION, VELOCITY, 
     *             VOLTAGE, CURRENT, MOTION_MAGIC, or FOLLOWER)
     * @param value The target value to set:
     *              - For PERCENT_OUTPUT: Output value between -1.0 and 1.0
     *              - For POSITION: Target position in sensor units
     *              - For VELOCITY: Target velocity in sensor units per 100ms
     *              - For VOLTAGE: Target voltage
     *              - For CURRENT: Target current in amps
     *              - For MOTION_MAGIC: Target position in sensor units
     *              - For FOLLOWER: Device ID of the motor to follow
     */
    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(new DutyCycleOut(value));
                break;
            case POSITION:
                motor.setControl(new PositionDutyCycle(value).withSlot(0));
                break;
            case VELOCITY:
                motor.setControl(new VelocityDutyCycle(value).withSlot(0));
                break;
            case VOLTAGE:
                motor.setControl(new DutyCycleOut(value));
                break;
            case CURRENT:
                if (isKraken) {
                    // Kraken X60 supports FOC (Field Oriented Control) for better current control
                    motor.setControl(new TorqueCurrentFOC(value));
                } else {
                    // For standard TalonFX, use duty cycle control since direct current control
                    // isn't available
                    motor.setControl(new DutyCycleOut(value));
                }
                break;
            case MOTION_MAGIC:
                motor.setControl(new MotionMagicDutyCycle(value).withSlot(0));
                break;
            case FOLLOWER:
                // Follow another Talon FX controller
                int deviceID = (int) value;
                motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID, false));
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
        motor.setControl(new DutyCycleOut(speed));
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
        switch (slotIdx) {
            case 0:
                var slot0Config = new Slot0Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot0Config, slotIdx);
                break;

            case 1:
                var slot1Config = new Slot1Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot1Config, slotIdx);
                break;

            case 2:
                var slot2Config = new Slot2Configs()
                        .withKP(kP)
                        .withKI(kI)
                        .withKD(kD)
                        .withKV(kF);
                motor.getConfigurator().apply(slot2Config, slotIdx);
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
        // return motor.getForwardLimit().getValueAsDouble() == 1;
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets the state of the reverse limit switch.
     * @return True if the reverse limit switch is closed, false otherwise
     */
    @Override
    public boolean getReverseLimitSwitch() {
        // return motor.getReverseLimit().getValueAsDouble() == 1;
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Sets this motor as a strict follower of another motor controller.
     * @param deviceID The ID of the motor controller to follow
     */
    @Override
    public void setStrictFollower(int deviceID) {
        // Set this motor as a follower using the Follower control request
        motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID, false)); // false = don't oppose master
                                                                                    // direction
    }

    /**
     * Configures hard limits for the TalonFX motor.
     * @param enableForward True to enable the forward hard limit, false otherwise
     * @param enableReverse True to enable the reverse hard limit, false otherwise
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
        var simState = motor.getSimState();
        simState.setForwardLimit(enableForward); // Set the initial state
        simState.setReverseLimit(enableReverse);
    }
}
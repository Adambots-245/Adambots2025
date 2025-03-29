package com.adambots.actuators;

/**
 * Enhanced base motor interface that supports advanced motor control features
 */
/**
 * Interface representing a base motor with various control modes and configuration options.
 */
public interface BaseMotor extends BaseActuator{

    /**
     * Control modes available for the motors.
     */
    public enum ControlMode {
        PERCENT_OUTPUT,
        POSITION,
        VELOCITY,
        VOLTAGE,
        CURRENT,
        MOTION_MAGIC,
        MOTION_MAGIC_FOC_TORQUE,
        FOLLOWER
    }

    /**
     * Sets the selected control mode output value.
     * 
     * @param mode The control mode to use.
     * @param value The setpoint value (interpretation depends on mode).
     */
    void set(ControlMode mode, double value);

    
    /**
     * Configures the PID values for closed-loop control.
     * 
     * @param slotIdx Gain slot index (0-3).
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kF Feed forward gain.
     */
    void setPID(int slotIdx, double kP, double kI, double kD, double kF);

    /**
     * Configures motion magic parameters for smooth motion control.
     * 
     * @param cruiseVelocity Cruise velocity in sensor units/100ms.
     * @param acceleration Acceleration in sensor units/100ms/sec.
     * @param jerk Optional jerk limit in sensor units/100ms/sec/sec.
     */
    void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk);

    /**
     * Configures current limits for the motor.
     * 
     * @param stallLimit Current limit when motor is stalled/not moving.
     * @param freeLimit Current limit when motor is free spinning.
     * @param limitRPM RPM threshold to switch between stall/free limits.
     */
    void configureCurrentLimits(double stallLimit, double freeLimit, double limitRPM);

    /**
     * Configures forward and reverse software limits.
     * 
     * @param forwardLimit The forward position limit in rotations.
     * @param reverseLimit The reverse position limit in rotations.
     * @param enable Whether to enable the software limits.
     */
    void configureSoftLimits(double forwardLimit, double reverseLimit, boolean enable);

    /**
     * Enables or disables software limits.
     * 
     * @param enable Whether to enable the software limits.
     */
    void enableSoftLimits(boolean enable);

    /**
     * Sets the motor inversion state.
     * 
     * @param inverted Whether to invert the motor direction.
     */
    void setInverted(boolean inverted);

    /**
     * Sets the motor brake mode.
     * 
     * @param brake Whether to enable brake mode. If false, coast mode is used.
     */
    void setBrakeMode(boolean brake);

    /**
     * Sets the motor position.
     * 
     * @param rotations The position to set in rotations.
     */
    void setPosition(double rotations);

    /**
     * Enables voltage compensation.
     * 
     * @param voltage The voltage to compensate to.
     */
    void enableVoltageCompensation(double voltage);

    /**
     * Gets the current motor position.
     * 
     * @return The current position in rotations.
     */
    double getPosition();

    /**
     * Gets the current motor velocity.
     * 
     * @return The current velocity in sensor units/100ms.
     */
    double getVelocity();

    /**
     * Gets the current motor acceleration.
     * 
     * @return The current acceleration in sensor units/100ms/sec.
     */
    double getAcceleration();

    /**
     * Gets the current motor draw.
     * 
     * @return The current draw in amperes.
     */
    double getCurrentDraw();

    /**
     * Gets the current motor output percent.
     * 
     * @return The output percent (-1.0 to 1.0).
     */
    double getOutputPercent();

    /**
     * Gets the current motor temperature.
     * 
     * @return The temperature in degrees Celsius.
     */
    double getTemperature();

    /**
     * Gets the state of the forward limit switch.
     * 
     * @return True if the forward limit switch is triggered, false otherwise.
     */
    boolean getForwardLimitSwitch();

    /**
     * Gets the state of the reverse limit switch.
     * 
     * @return True if the reverse limit switch is triggered, false otherwise.
     */
    boolean getReverseLimitSwitch();

    /**
     * Sets the motor to follow another motor.
     * 
     * @param deviceID The device ID of the motor to follow.
     */
    void setStrictFollower(int deviceID);

    /**
     * Configures the hardware limit switches.
     * 
     * @param enableForward Whether to enable the forward limit switch.
     * @param enableReverse Whether to enable the reverse limit switch.
     * @param forwardValue Value to reset the encoder when the forward limit switch is hit
     * @param reverseValue Value to reset the encoder when the reverse limit switch is hit
     */
    void configureHardLimits(boolean enableForward, boolean enableReverse, double forwardValue, double reverseValue);

    
}
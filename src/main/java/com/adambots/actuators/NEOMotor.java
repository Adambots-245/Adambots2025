package com.adambots.actuators;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Implementation for REV NEO and NEO Vortex motors using SPARK MAX controller
 */
public class NEOMotor implements BaseMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig config;

    /**
     * Constructs a NEOMotor object.
     *
     * @param portNum The port number for the motor.
     * @param brushed True if the motor is brushed, false if brushless.
     */
    public NEOMotor(int portNum, boolean brushed) {
        motor = new SparkMax(portNum, brushed ? MotorType.kBrushed : MotorType.kBrushless);
        config = new SparkMaxConfig();

        // Get the built-in encoder
        encoder = motor.getEncoder();

        // Get the closed loop controller
        closedLoopController = motor.getClosedLoopController();

        // Default configuration
        config.voltageCompensation(12.0);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor control mode and value.
     *
     * @param mode  The control mode to set.
     * @param value The value to set for the control mode.
     */
    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.set(value);
                break;
            case POSITION:
                closedLoopController.setReference(value, SparkBase.ControlType.kPosition);
                break;
            case VELOCITY:
                closedLoopController.setReference(value, SparkBase.ControlType.kVelocity);
                break;
            case VOLTAGE:
                motor.setVoltage(value);
                break;
            case CURRENT:
                closedLoopController.setReference(value, SparkBase.ControlType.kCurrent);
                break;
            case MOTION_MAGIC:
                // Use Smart Motion for motion profiling
                closedLoopController.setReference(value, SparkBase.ControlType.kSmartMotion);
                break;
            case FOLLOWER:
                config.follow((int) value);
                break;
        }
    }

    /**
     * Sets the motor speed.
     *
     * @param speed The speed to set for the motor.
     */
    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    /**
     * Sets the PIDF constants for the motor's closed loop control.
     *
     * @param slotIdx The slot index for the PIDF constants.
     * @param kP      The proportional gain.
     * @param kI      The integral gain.
     * @param kD      The derivative gain.
     * @param kF      The feedforward gain.
     */
    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        config.closedLoop.pidf(kP, kI, kD, kF);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Configures motion magic settings for motion profiling.
     *
     * @param cruiseVelocity The cruise velocity for motion magic.
     * @param acceleration   The acceleration for motion magic.
     * @param jerk           The jerk for motion magic.
     */
    @Override
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        config.closedLoop.smartMotion
                .maxVelocity(cruiseVelocity)
                .maxAcceleration(acceleration);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Configures current limits for the motor.
     *
     * @param stallLimit The stall current limit.
     * @param freeLimit  The free current limit.
     * @param limitRPM   The RPM limit for current limiting.
     */
    @Override
    public void configureCurrentLimits(double stallLimit, double freeLimit, double limitRPM) {
        config.smartCurrentLimit((int) stallLimit, (int) freeLimit, (int) limitRPM);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Configures soft limits for the motor.
     *
     * @param forwardLimit  The forward soft limit value.
     * @param reverseLimit  The reverse soft limit value.
     * @param enable        True to enable soft limits, false to disable.
     */
    @Override
    public void configureSoftLimits(double forwardLimit, double reverseLimit, boolean enable) {
        config.softLimit.forwardSoftLimitEnabled(enable).forwardSoftLimit(forwardLimit);
        config.softLimit.reverseSoftLimitEnabled(enable).reverseSoftLimit(reverseLimit);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Enables or disables soft limits for the motor.
     *
     * @param enable True to enable soft limits, false to disable.
     */
    @Override
    public void enableSoftLimits(boolean enable) {
        config.softLimit.forwardSoftLimitEnabled(enable);
        config.softLimit.reverseSoftLimitEnabled(enable);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets the inversion of the motor.
     *
     * @param inverted True to invert the motor, false to not invert.
     */
    @Override
    public void setInverted(boolean inverted) {
        config.inverted(inverted);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets the brake mode of the motor.
     *
     * @param brake True to set brake mode, false to set coast mode.
     */
    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets the encoder position to a specific value in rotations.
     *
     * @param rotations The position to set the encoder to in rotations.
     */
    @Override
    public void setPosition(double rotations) {
        encoder.setPosition(rotations);
    }

    /**
     * Enables voltage compensation for the motor.
     *
     * @param voltage The voltage to compensate to.
     */
    @Override
    public void enableVoltageCompensation(double voltage) {
        config.voltageCompensation(voltage);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Gets the current position of the motor's encoder in rotations.
     *
     * @return The current position of the encoder in rotations.
     */
    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Gets the current velocity of the motor in RPM.
     *
     * @return The current velocity of the motor in RPM.
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the current acceleration of the motor.
     * <p>
     * Note: NEO motors do not directly support acceleration measurement.
     *
     * @return 0.0 as acceleration is not directly supported.
     */
    @Override
    public double getAcceleration() {
        // NEO does not directly support acceleration measurement
        return 0.0;
    }

    /**
     * Gets the current draw of the motor in Amperes.
     *
     * @return The current draw of the motor in Amperes.
     */
    @Override
    public double getCurrentDraw() {
        return motor.getOutputCurrent();
    }

    /**
     * Gets the output percentage of the motor controller.
     *
     * @return The output percentage as a decimal value between -1 and 1.
     */
    @Override
    public double getOutputPercent() {
        return motor.get();
    }

    /**
     * Gets the temperature of the motor in degrees Celsius.
     *
     * @return The temperature of the motor in degrees Celsius.
     */
    @Override
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * Gets the state of the forward limit switch.
     *
     * @return True if the forward limit switch is pressed, false otherwise.
     */
    @Override
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    /**
     * Gets the state of the reverse limit switch.
     *
     * @return True if the reverse limit switch is pressed, false otherwise.
     */
    @Override
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    /**
     * Sets the motor to strictly follow another motor controller.
     *
     * @param deviceID The device ID of the motor controller to follow.
     */
    @Override
    public void setStrictFollower(int deviceID) {
        config.follow(deviceID);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Configures hard limits using limit switches.
     *
     * @param enableForward  True to enable the forward limit switch, false otherwise.
     * @param enableReverse  True to enable the reverse limit switch, false otherwise.
     */
    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse, double forwardValue, double reverseValue) {
        config.limitSwitch.setSparkMaxDataPortConfig()
                .forwardLimitSwitchEnabled(enableForward)
                .reverseLimitSwitchEnabled(enableReverse);

        if (enableForward) {
            config.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        }
        if (enableReverse) {
            config.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        }
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
}
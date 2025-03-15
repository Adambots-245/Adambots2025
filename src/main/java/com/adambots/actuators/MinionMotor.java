package com.adambots.actuators;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;

/**
 * Implementation of BaseMotor for Minion Motor using TalonFXS motor controller
 */
public class MinionMotor implements BaseMotor {
    private final TalonFXS motor;
    private final TalonFXSConfigurator configurator;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);
    private final Follower followerRequest = new Follower(0, false);
    
    /**
     * Constructor for MinionMotor using TalonFXS
     * 
     * @param deviceID The CAN ID of the motor controller
     */
    public MinionMotor(int deviceID) {
        motor = new TalonFXS(deviceID);
        configurator = motor.getConfigurator();
    }
    
    /**
     * Constructor for MinionMotor using TalonFXS with specific CAN bus
     * 
     * @param deviceID The CAN ID of the motor controller
     * @param canBus The name of the CAN bus
     */
    public MinionMotor(int deviceID, String canBus) {
        motor = new TalonFXS(deviceID, canBus);
        configurator = motor.getConfigurator();
    }

    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(percentRequest.withOutput(value));
                break;
            case POSITION:
                motor.setControl(positionRequest.withPosition(value));
                break;
            case VELOCITY:
                motor.setControl(velocityRequest.withVelocity(value));
                break;
            case VOLTAGE:
                motor.setControl(voltageRequest.withOutput(value));
                break;
            case CURRENT:
                // TalonFXS doesn't have a direct current control mode in Phoenix 6
                // This could be implemented with a custom control request if needed
                System.err.println("CURRENT control mode not directly supported by TalonFXS");
                break;
            case MOTION_MAGIC:
                motor.setControl(motionMagicRequest.withPosition(value));
                break;
            case FOLLOWER:
                // In Phoenix 6, Follower requires device ID
                // value here is assumed to be the device ID to follow
                motor.setControl(new Follower((int)value, false));
                break;
            default:
                System.err.println("Unsupported control mode");
                break;
        }
    }

    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        // In Phoenix 6, slot configuration is done differently
        Slot0Configs slot0Configs = new Slot0Configs();
        
        // Read current configuration first to avoid overwriting other settings
        configurator.refresh(slot0Configs);
        
        // Update PID values
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kV = kF;  // kV is the feed forward in Phoenix 6
        
        // Apply configuration
        configurator.apply(slot0Configs);
    }

    @Override
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        MotionMagicConfigs configs = new MotionMagicConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Set new values
        configs.MotionMagicCruiseVelocity = cruiseVelocity;
        configs.MotionMagicAcceleration = acceleration;
        configs.MotionMagicJerk = jerk;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void configureCurrentLimits(double stallLimit, double freeLimit, double limitRPM) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Set new values - note that Phoenix 6 has different terminology
        configs.StatorCurrentLimit = stallLimit;
        configs.StatorCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = freeLimit;
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLowerLimit = freeLimit * 0.8; // Lower limit after time threshold
        configs.SupplyCurrentLowerTime = 0.1; // Time to exceed threshold before lowering limit
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void configureSoftLimits(double forwardLimit, double reverseLimit, boolean enable) {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Set new values
        configs.ForwardSoftLimitThreshold = forwardLimit;
        configs.ForwardSoftLimitEnable = enable;
        configs.ReverseSoftLimitThreshold = reverseLimit;
        configs.ReverseSoftLimitEnable = enable;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void enableSoftLimits(boolean enable) {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Update enable values only
        configs.ForwardSoftLimitEnable = enable;
        configs.ReverseSoftLimitEnable = enable;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void setInverted(boolean inverted) {
        MotorOutputConfigs configs = new MotorOutputConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Update inversion setting
        configs.Inverted = inverted ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
                                     com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        MotorOutputConfigs configs = new MotorOutputConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Update neutral mode setting
        configs.NeutralMode = brake ? com.ctre.phoenix6.signals.NeutralModeValue.Brake : 
                                     com.ctre.phoenix6.signals.NeutralModeValue.Coast;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void setPosition(double rotations) {
        // In Phoenix 6, we set the position through the configurator
        motor.setPosition(rotations);
    }

    @Override
    public void enableVoltageCompensation(double voltage) {
        VoltageConfigs configs = new VoltageConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Set voltage compensation
        configs.PeakForwardVoltage = voltage;
        configs.PeakReverseVoltage = -voltage;
        configs.SupplyVoltageTimeConstant = 0.1; // Filter time constant
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public double getPosition() {
        // In Phoenix 6, getPosition returns a StatusSignal<Double> representing rotations
        var positionSignal = motor.getPosition();
        positionSignal.refresh(); // Make sure we have the latest value
        return positionSignal.getValue().magnitude(); // Value is in rotations which matches our interface
    }

    @Override
    public double getVelocity() {
        // In Phoenix 6, getVelocity returns rotations per second
        var velocitySignal = motor.getVelocity();
        velocitySignal.refresh();
        return velocitySignal.getValue().magnitude(); // Value is in RPS
    }

    @Override
    public double getAcceleration() {
        var accelerationSignal = motor.getAcceleration();
        accelerationSignal.refresh();
        return accelerationSignal.getValue().magnitude(); // Value is in RPS/s
    }

    @Override
    public double getCurrentDraw() {
        var currentSignal = motor.getStatorCurrent();
        currentSignal.refresh();
        return currentSignal.getValue().magnitude(); // Value is in amperes
    }

    @Override
    public double getOutputPercent() {
        var outputSignal = motor.getDutyCycle();
        outputSignal.refresh();
        return outputSignal.getValue().doubleValue(); // Value is from -1.0 to 1.0
    }

    @Override
    public double getTemperature() {
        var tempSignal = motor.getDeviceTemp();
        tempSignal.refresh();
        return tempSignal.getValue().magnitude(); // Value is in Celsius
    }

    @Override
    public boolean getForwardLimitSwitch() {
        var limitSignal = motor.getForwardLimit();
        limitSignal.refresh();
        return limitSignal.getValue().equals(ForwardLimitValue.ClosedToGround); // Returns true if switch is closed
    }

    @Override
    public boolean getReverseLimitSwitch() {
        var limitSignal = motor.getReverseLimit();
        limitSignal.refresh();
        return limitSignal.getValue().equals(ReverseLimitValue.ClosedToGround); // Returns true if switch is closed
    }

    @Override
    public void setStrictFollower(int deviceID) {
        // Set this motor to follow another device
        motor.setControl(new Follower(deviceID, false));
    }

    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse, double forwardValue, double reverseValue) {
        HardwareLimitSwitchConfigs configs = new HardwareLimitSwitchConfigs();
        
        // Read current configuration
        configurator.refresh(configs);
        
        // Configure limits
        configs.ForwardLimitEnable = enableForward;
        configs.ForwardLimitAutosetPositionEnable = enableForward;
        configs.ForwardLimitAutosetPositionValue = forwardValue;
        
        configs.ReverseLimitEnable = enableReverse;
        configs.ReverseLimitAutosetPositionEnable = enableReverse;
        configs.ReverseLimitAutosetPositionValue = reverseValue;
        
        // Apply configuration
        configurator.apply(configs);
    }

    @Override
    public void set(double speed) {
        set(ControlMode.PERCENT_OUTPUT, speed);
    }
}
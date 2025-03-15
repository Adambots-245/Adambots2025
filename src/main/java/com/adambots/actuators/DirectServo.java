package com.adambots.actuators;

import edu.wpi.first.wpilibj.Servo;

public class DirectServo implements BaseServo {
    private final Servo servo;
    private final ServoMode mode;

    // Pulse width constants (microseconds)
    private static final double SERVO_MIN_PULSE_WIDTH_US = 600.0;
    private static final double SERVO_MAX_PULSE_WIDTH_US = 2400.0;

    // Configurable angle range
    private double minAngle = 0.0;
    private double maxAngle = 180.0;  // Default for standard servos
    private double centerAngle = 90.0;

    /**
     * Creates a new DirectServo connected to a PWM port
     * @param channel The PWM channel this servo is connected to
     * @param mode The servo mode (CONTINUOUS_ROTATION or ANGULAR)
     */
    public DirectServo(int channel, ServoMode mode) {
        this.servo = new Servo(channel);
        this.mode = mode;
        updateCenterAngle();
    }

    /**
     * Creates a new DirectServo with custom angle range
     * @param channel The PWM channel this servo is connected to
     * @param mode The servo mode (CONTINUOUS_ROTATION or ANGULAR)
     * @param minAngle Minimum angle in degrees
     * @param maxAngle Maximum angle in degrees
     */
    public DirectServo(int channel, ServoMode mode, double minAngle, double maxAngle) {
        this.servo = new Servo(channel);
        this.mode = mode;
        setAngleRange(minAngle, maxAngle);
    }

    /**
     * Sets the angle range for the servo
     * @param minAngle Minimum angle in degrees
     * @param maxAngle Maximum angle in degrees
     */
    public void setAngleRange(double minAngle, double maxAngle) {
        if (maxAngle <= minAngle) {
            throw new IllegalArgumentException("maxAngle must be greater than minAngle");
        }
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        updateCenterAngle();
    }

    private void updateCenterAngle() {
        this.centerAngle = (maxAngle + minAngle) / 2.0;
    }

    @Override
    public ServoMode getMode() {
        return mode;
    }

    @Override
    public void turnCounterclockwise() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(1.0);  // Full speed CCW
        } else {
            servo.setAngle(maxAngle);  // Max angle
        }
    }

    @Override
    public void turnClockwise() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(0.0);  // Full speed CW
        } else {
            servo.setAngle(minAngle);  // Min angle
        }
    }

    @Override
    public void stop() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(0.5);  // Center position stops rotation
        } else {
            servo.setAngle(centerAngle);  // Center position
        }
    }

    @Override
    public void set(double speed) {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            // Map -1.0 to 1.0 to 0.0 to 1.0 range for WPILib Servo
            double mappedSpeed = (speed + 1.0) / 2.0;
            servo.set(mappedSpeed);
        } else {
            throw new UnsupportedOperationException("Speed control not supported in ANGULAR mode");
        }
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        // Convert microseconds to position (0-1 range)
        // WPILib expects 0.6ms-2.4ms range mapped to 0-1
        double position = (pulseWidth - SERVO_MIN_PULSE_WIDTH_US) / (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US);
        position = Math.min(1.0, Math.max(0.0, position));
        servo.set(position);
    }

    @Override
    public double getCurrent() {
        // WPILib Servo class doesn't provide current monitoring
        return 0.0;
    }

    @Override
    public void setAngle(double degrees) {
        if (mode == ServoMode.ANGULAR) {
            // Clamp to valid range
            degrees = Math.min(maxAngle, Math.max(minAngle, degrees));

            // Map the custom angle range to WPILib's 0-180 range
            double normalizedAngle = (degrees - minAngle) / (maxAngle - minAngle) * 180.0;
            servo.setAngle(normalizedAngle);
        } else {
            throw new UnsupportedOperationException("Angle control not supported in CR mode");
        }
    }

    /**
     * Gets the raw position (0 to 1 range)
     * @return The current position
     */
    public double getPosition() {
        return servo.get();
    }

    /**
     * Gets the current angle in degrees (only valid in ANGULAR mode)
     * @return The current angle in the configured range
     */
    public double getAngle() {
        if (mode == ServoMode.ANGULAR) {
            // Convert from WPILib's 0-180 range back to our custom range
            double wpiLibAngle = servo.getAngle();
            return minAngle + (wpiLibAngle / 180.0) * (maxAngle - minAngle);
        }
        throw new UnsupportedOperationException("Angle reading not supported in CR mode");
    }

    /**
     * Gets the minimum configured angle
     * @return The minimum angle in degrees
     */
    public double getMinAngle() {
        return minAngle;
    }

    /**
     * Gets the maximum configured angle
     * @return The maximum angle in degrees
     */
    public double getMaxAngle() {
        return maxAngle;
    }

    /**
     * Gets the center angle
     * @return The center angle in degrees
     */
    public double getCenterAngle() {
        return centerAngle;
    }
}
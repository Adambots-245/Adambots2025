package com.adambots.actuators;

/**
 * Interface for a servo motor that can operate in either Continuous Rotation (CR)
 * or Angular (Position) mode.
 */
public interface BaseServo extends BaseActuator{
    /**
     * Sets the operating mode of the servo.
     */
    enum ServoMode {
        CONTINUOUS_ROTATION,
        ANGULAR
    }

    /**
     * Gets the current operating mode of the servo.
     * @return The current ServoMode
     */
    ServoMode getMode();

    /**
     * For CR mode: Turns the servo counterclockwise
     * For Angular mode: Sets position to maximum angle
     */
    void turnCounterclockwise();

    /**
     * For CR mode: Turns the servo clockwise
     * For Angular mode: Sets position to minimum angle
     */
    void turnClockwise();

    /**
     * For CR mode: Stops rotation
     * For Angular mode: Sets to center position
     */
    void stop();

    /**
     * Sets the raw pulse width of the servo.
     * @param pulseWidth The pulse width in microseconds
     */
    void setPulseWidth(int pulseWidth);

    /**
     * Gets the current being drawn by the servo.
     * @return The current in amps
     */
    double getCurrent();

    /**
     * For Angular mode: Sets the angle of the servo
     * For CR mode: This will throw UnsupportedOperationException
     * @param degrees The angle in degrees (0-355 for Axon Max+)
     */
    default void setAngle(double degrees) {
        throw new UnsupportedOperationException("Angle control not supported in current mode");
    }

    /**
     * For CR mode: Sets the speed of rotation
     * For Angular mode: This will throw UnsupportedOperationException
     * @param speed Speed from -1.0 (full CCW) to 1.0 (full CW)
     */
    @Override
    default void set(double speed) {
        throw new UnsupportedOperationException("Speed control not supported in current mode");
    }
}

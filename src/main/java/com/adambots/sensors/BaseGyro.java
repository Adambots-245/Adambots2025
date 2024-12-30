package com.adambots.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

public interface BaseGyro {

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in degrees
     * <p>
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in degrees
     */
    double getContinuousYawDeg();

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in radians
     * <p>
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in radians
     */
    double getContinuousYawRad();

    Rotation2d getContinuousYawRotation2d();

    void offsetYawByAngle (double offsetDeg);

    /**
     * Zeros gyroscope yaw
     */
    void resetYaw();

    /**
     * Resets the yaw of the gyroscope to the specified value in degrees
     */
    void resetYawToAngle(double offsetDeg);

    /**
     * Returns the measured roll value of the gyroscope in degrees
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation
     * @return Roll value of gyroscope in degrees
     */
    double getPitch();

    /**
     * Returns the measured pitch value of the gyroscope in degrees
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation
     * @return Pitch value of gyroscope in degrees
     */
    double getRoll();

}
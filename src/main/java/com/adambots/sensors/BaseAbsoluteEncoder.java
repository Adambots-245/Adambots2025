package com.adambots.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

public interface BaseAbsoluteEncoder {

    /**
     * Returns the discrete (does not continue past 360) value of the encoder in degrees
     * @return Discrete value of encoder in degrees
     */
    double getAbsolutePositionDegrees();

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    double getAbsolutePositionRadians();

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    Rotation2d getAbsolutePositionRotation2D();

}
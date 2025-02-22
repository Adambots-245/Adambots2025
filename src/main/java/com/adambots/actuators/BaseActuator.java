// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

/**
 * Base interface for all actuators.
 */
public interface BaseActuator {
    /**
     * Sets basic percent output (-1.0 to 1.0).
     * 
     * @param speed The speed to set (-1.0 to 1.0). For servos, -1 is full CCW, 1 is full CW.
     */
    void set(double speed);

}

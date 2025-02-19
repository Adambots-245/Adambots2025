// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

/** Add your docs here. */
public interface BaseServo {

    void turnCounterclockwise();

    void turnClockwise();

    void stop();

    void setPulseWidth(int pulseWidth);

    double getCurrent();

}

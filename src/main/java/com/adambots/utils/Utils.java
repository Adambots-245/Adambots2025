// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility functions for the robot
 */
public class Utils {
    /**
     * Returns {@code True} if the robot is on the red alliance according to
     * driverstation, {@code False} otherwise
     *
     * @return Whether the robot is on the red alliance
     */
    public static boolean isOnRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}

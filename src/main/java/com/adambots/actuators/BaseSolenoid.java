// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

public interface BaseSolenoid {

    /**
     * Enable the solenoid
     */
    void enable();

    /**
     * Disable the solenoid
     */
    void disable();

    /**
     * Toggle the state - enable if disabled or disable if enabled
     */
    void toggle();

    /**
     * Get the current state
     * @return true if enabled, false otherwise
     */
    boolean get();

    /**
     * Set the current state - true to enable and false to disable
     * @param value
     */
    void set(boolean value);
}

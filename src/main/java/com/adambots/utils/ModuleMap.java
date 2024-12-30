// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Source: Team 1596

package com.adambots.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.subsystems.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Contains functions to convert a {@link Map} with {@link ModulePosition} keys
 * to and from arrays so
 * that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {
  /**
   * Returns the values from the map as an {@code Array} in the same order as in
   * the {@link ModulePosition} enum. This avoids any error in the order of the motors and
   * modules.
   * <p>
   * Useful when a WPILib swerve function requires an array as input.
   *
   * @param swerveModulePositions
   * An array of the class to output an array of, e.g. {@code moduleTranslations.valuesArray(new SwerveModulePosition[0])}. 
   * Required because Java can't make an array of generics.
   */
  public static SwerveModulePosition[] orderedModulePositions(Map<ModulePosition, SwerveModule> map) {
    ArrayList<SwerveModulePosition> list = new ArrayList<>();

    for (ModulePosition i : ModulePosition.values()) {
      list.add(map.get(i).getPosition());
    }

    return (SwerveModulePosition[]) list.toArray(new SwerveModulePosition[0]);
  }

  /**
   * Sets desired states of the swerve modules using an {@code Array} in the same order as in
   * the {@link ModulePosition} enum. This avoids any error in the order of the motors and
   * modules.
   * <p>
   * Useful when setting module states given an array of states
   *
   * @param swerveModules
   * The {@code HashMap} of the swerve modules that are to be set
   * @param swerveModuleStates
   * The {@code Array} of SwerveModuleStates to be applied to the swerve modules
   */
  public static void setDesiredState(HashMap<ModulePosition, SwerveModule> swerveModules, SwerveModuleState[] swerveModuleStates) {
    int idx = 0;

    for (ModulePosition i : ModulePosition.values()) {
      swerveModules.get(i).setDesiredState(swerveModuleStates[idx++]);
    }
  }

  /**
   * Gets the current states of a {@code HashMap} of swerve modules as an {@code Array} in the same order as in
   * the {@link ModulePosition} enum. This avoids any error in the order of the motors and
   * modules.
   *
   * @param swerveModules
   * The {@code HashMap} of the swerve modules to get the states of
   * @return The current SwerveModuleStates as an {@code Array} 
   */
  public static SwerveModuleState[] orderedModuleStates(HashMap<ModulePosition, SwerveModule> swerveModules) {
    SwerveModuleState[] arr = new SwerveModuleState[4];
    int idx = 0;

    for (ModulePosition i : swerveModules.keySet()) {
      arr[idx++] = swerveModules.get(i).getState();
    }
    return arr;
  }

  /**
   * Sets the motor speeds of each swerve module to 0.
   *
   * @param swerveModules
   * The {@code HashMap} of swerve modules to be set
   */
  public static void stopModules(HashMap<ModulePosition, SwerveModule> swerveModules) {
    for (ModulePosition i : swerveModules.keySet()) {
      swerveModules.get(i).stopMotors();
    }
  }
}

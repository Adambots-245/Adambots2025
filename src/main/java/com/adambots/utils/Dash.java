// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

import com.adambots.Constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Dashboard utility to add values to Shuffleboard and automatically update.
 * 
 * Dash.add("Speed", () -> {
 *  return speed;
 * });
 */
public class Dash {
    private static ShuffleboardTab debugTab = Shuffleboard.getTab(Constants.kDefaultShuffleboardTab);


    private Dash(){
        throw new UnsupportedOperationException("Not meant to be instantiated. Utility class");
    }

    public static void add(String name, DoubleSupplier dval){
        debugTab.addDouble(name, dval);
    }

    public static void add(String name, LongSupplier ival){
        debugTab.addInteger(name, ival);
    }
    
    public static void add(String name, BooleanSupplier bval){
        debugTab.addBoolean(name, bval);
    }
    public static void add(String name, String sval){
        debugTab.add(name, sval);
    }
}

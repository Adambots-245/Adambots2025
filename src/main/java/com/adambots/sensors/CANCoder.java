/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Generic Absolute Encoder sensor to hide actual implementation and ensure uniform values across subsystems
 */
public class CANCoder implements BaseAbsoluteEncoder {
    private CANcoder encoder;
    
    public CANCoder (int port){
        this.encoder = new CANcoder(port); //Defining the encoder using the port passed in
    }

    /**
     * Returns the discrete (does not continue past 360) value of the encoder in degrees
     * @return Discrete value of encoder in degrees
     */
    @Override
    public double getAbsolutePositionDegrees () {
        return encoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    @Override
    public double getAbsolutePositionRadians () {
        return Units.degreesToRadians(getAbsolutePositionDegrees());
    }

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    @Override
    public Rotation2d getAbsolutePositionRotation2D () {
        return new Rotation2d(Units.degreesToRadians(getAbsolutePositionDegrees()));
    }
}

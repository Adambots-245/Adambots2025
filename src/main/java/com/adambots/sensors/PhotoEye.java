
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Generic digital sensor to hide actual implementation
 */
public class PhotoEye implements BaseProximitySensor {
    private DigitalInput sensor;
    private Boolean inverted;

    /**
     * Define a PhotoEye or limit switch on a given DigitalInput port. 
     * If you set up the sensor to return true when not detecting as to ensure a non-critical failure mode, 
     * you can define it as inverted to maintain the expected (reads true when detecting) return in code.
     * <p>
     * (digital inputs return true when disconnected)
     * @param port
     * The digital input port of the sensor
     * @param inverted
     * Whether or not to invert the sensor return
     */
    public PhotoEye (int port, boolean inverted){
        this.sensor = new DigitalInput(port);
        this.inverted = inverted;
    }

    /**
     * Returns the detection of the sensor with regards to whether or not the sensor is defined as inverted
     *
     * @return Whether or not the sensor is detecting something
     */
    public boolean isDetecting(){
        if (inverted) return !sensor.get();
        return sensor.get(); 
    }
}
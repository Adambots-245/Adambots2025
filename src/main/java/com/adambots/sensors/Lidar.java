/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Generic lidar sensor to hide actual implementation
 */
public class Lidar implements BaseDistanceSensor {
    private Counter counter;
    private DigitalInput _source = null;
	
	public Lidar(int dioPortNumber) {
		super();

        _source = new DigitalInput(dioPortNumber);

		counter = new Counter(_source);

	    counter.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
	    counter.setSemiPeriodMode(true);
	    counter.reset();
    }
    
	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in cm
	 */
	@Override
	public double getDistanceInCentimeters() {
		double cm;

		// getPeriod returns time in seconds. The hardware resolution is microseconds.
		// The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return cm;
	}

	/**
	 * Take a measurement and return the distance in inches
     * 
	 * @return Distance in inches
	 */
	@Override
	public double getDistanceInInches() {
		return getDistanceInCentimeters() / 2.54;
    }

	@Override
	public double getDistanceInFeet() {
		return getDistanceInInches() / 12.0;
	}

	
}

package com.adambots.sensors;

import com.ctre.phoenix6.hardware.CANrange;

/**
 * Class to represent the CANRange sensor that measures distance from an object.
 * Implements the BaseDistanceSensor interface.
 */
public class CANRangeSensor implements BaseDistanceSensor {

    private CANrange canRangeSensor;

    /**
     * Constructor for CANRangeSensor.
     * @param deviceId The device ID of the CANRange sensor.
     */
    public CANRangeSensor(int deviceId) {
        canRangeSensor = new CANrange(deviceId);
    }

    @Override
    /**
     * Returns the distance in centimeters.
     * @return Distance in centimeters.
     */
    public double getDistanceInCentimeters() {
        return canRangeSensor.getDistance().getValueAsDouble() * 100.0; // Convert meters to centimeters
    }

    @Override
    /**
     * Returns the distance in inches.
     * @return Distance in inches.
     */
    public double getDistanceInInches() {
        return canRangeSensor.getDistance().getValueAsDouble() * 39.37; // Convert meters to inches
    }

    @Override
    /**
     * Returns the distance in feet.
     * @return Distance in feet.
     */
    public double getDistanceInFeet() {
        return canRangeSensor.getDistance().getValueAsDouble() * 3.281; // Convert meters to feet
    }

}
package com.adambots.sensors;

public interface BaseDistanceSensor {

    /**
     * Take a measurement and return the distance in cm
     * 
     * @return Distance in cm
     */
    double getDistanceInCentimeters();

    /**
     * Take a measurement and return the distance in inches
     * 
     * @return Distance in inches
     */
    double getDistanceInInches();

    /**
     * Take a measurement and return the distance in feet
     * @return
     */
    double getDistanceInFeet();

}
package com.adambots.sensors;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Class to represent the CANRange sensor that measures distance from an object.
 * Implements the BaseDistanceSensor interface.
 */
public class CANRangeSensor implements BaseDistanceSensor {

    private CANrange canRangeSensor;
    private CANrangeSimState simState;
    private final boolean isSim;

    /**
     * Constructor for CANRangeSensor.
     * @param deviceId The device ID of the CANRange sensor.
     */
    public CANRangeSensor(int deviceId) {
        canRangeSensor = new CANrange(deviceId);
        isSim = RobotBase.isSimulation();
        
        if (isSim) {
            simState = canRangeSensor.getSimState();
            // Set default simulation values
            simState.setDistance(0.0);  // Start at 0 meters
        }
    }

    @Override
    public double getDistanceInCentimeters() {
        return canRangeSensor.getDistance().getValueAsDouble() * 100.0; // Convert meters to centimeters
    }

    @Override
    public double getDistanceInInches() {
        return canRangeSensor.getDistance().getValueAsDouble() * 39.37; // Convert meters to inches
    }

    @Override
    public double getDistanceInFeet() {
        return canRangeSensor.getDistance().getValueAsDouble() * 3.281; // Convert meters to feet
    }

    /**
     * Sets the simulated distance for testing.
     * Only works in simulation mode.
     * @param distanceInMeters The distance to simulate in meters
     */
    public void setSimulatedDistance(double distanceInMeters) {
        if (isSim) {
            simState.setDistance(distanceInMeters);
        }
    }

    /**
     * Returns whether we are running in simulation mode
     * @return true if in simulation
     */
    public boolean isSim() {
        return isSim;
    }
    
    /**
     * Gets the raw distance in meters.
     * Useful for testing.
     * @return Distance in meters
     */
    public double getRawDistance() {
        return canRangeSensor.getDistance().getValueAsDouble();
    }
}
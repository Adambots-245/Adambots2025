// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class LimitSwitch implements BaseProximitySensor {
    private DigitalInput sensor;
    private boolean inverted;

    public LimitSwitch(int port, boolean inverted) {
        this.sensor = new DigitalInput(port);
        this.inverted = inverted;
    }

    @Override
    public boolean isDetecting() {
        if (inverted) return !sensor.get();
        return sensor.get(); 
    }
}

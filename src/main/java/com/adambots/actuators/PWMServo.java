// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class PWMServo implements BaseServo {

    Servo servo;

    public PWMServo(int portNum) {
        servo = new Servo(portNum);
    }

    @Override
    public void turnCounterclockwise() {
        servo.setSpeed(1);
    }

    @Override
    public void turnClockwise() {
        servo.setSpeed(-1);
    }

    @Override
    public void stop() {
        servo.setSpeed(0);
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPulseWidth'");
    }

    @Override
    public double getCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getCurrent'");
    }

}
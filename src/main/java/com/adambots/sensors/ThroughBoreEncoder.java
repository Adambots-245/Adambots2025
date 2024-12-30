// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ThroughBoreEncoder implements BaseAbsoluteEncoder {
    private DutyCycleEncoder encoder;

    public ThroughBoreEncoder(int encoderPort) {
        encoder = new DutyCycleEncoder(encoderPort);
    }

    @Override
    public double getAbsolutePositionDegrees() {
        return encoder.get() * 360.0;
    }

    @Override
    public double getAbsolutePositionRadians() {
        return Math.toRadians(getAbsolutePositionDegrees());
    }

    @Override
    public Rotation2d getAbsolutePositionRotation2D() {
        return new Rotation2d(getAbsolutePositionRadians());
    }
}

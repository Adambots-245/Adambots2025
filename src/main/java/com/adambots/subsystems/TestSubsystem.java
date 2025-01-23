// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

BaseMotor KrakenMotor;
double speed;

public class TestSubsystem extends SubsystemBase {
  /** Creates a new TestSubsystem. */
  public TestSubsystem(BaseMotor KrakenMotor) {
    this.KrakenMotor = KrakenMotor;

  }

  public setMotorSpeed(double speed) {
    KrakenMotor.setSpeed(speed);

  }

  public stopMotor() {
    KrakenMotor.setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

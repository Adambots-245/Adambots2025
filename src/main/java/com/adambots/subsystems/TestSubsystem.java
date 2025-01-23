// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TestSubsystem extends SubsystemBase {

  BaseMotor KrakenMotor;
  double speed;


  /** Creates a new TestSubsystem. */
  public TestSubsystem(BaseMotor KrakenMotor) {
    this.KrakenMotor = KrakenMotor;

  }

  public void setMotorSpeed(double speed) {
    KrakenMotor.set(speed);

  }

  public void stopMotor() {
    KrakenMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

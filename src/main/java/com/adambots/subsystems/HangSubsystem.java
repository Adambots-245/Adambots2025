// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;
import com.adambots.actuators.BaseServo;
import com.adambots.actuators.BaseSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  BaseMotor hangMotor;
  BaseSolenoid hangSolenoid;
  BaseServo hangServo;
  double hangMotorSpeed = 0;

  public HangSubsystem(BaseMotor hangMotor, BaseSolenoid hangSolenoid, BaseServo hangServo) {
    this.hangMotor = hangMotor;
    this.hangSolenoid = hangSolenoid;
    this.hangServo = hangServo;
  }

  public void setMotorSpeed(double newMotorSpeed) {
    hangMotorSpeed = newMotorSpeed;
  }

  public double getMotorPosition() {
    return Math.abs(hangMotor.getPosition()); // Return absolute value so motor inversion doesn't affect failsafes
  }

  public void resetEncoders() {
    hangMotor.setPosition(0);
  }

  public void setSolenoids(Boolean active) {
    hangSolenoid.toggle();
  }

  public void releaseServo() {
    hangServo.setAngle(50);
  }

  @Override
  public void periodic() {

    // failSafes();
    hangMotor.set(hangMotorSpeed);

    SmartDashboard.putBoolean("Hang/Solinoid", hangSolenoid.get());

  }

  // public void failSafes() {
  // // if (getLeftMotorPosition() < 0 && leftHangMotorSpeed < 0) {
  // // leftHangMotorSpeed = 0;
  // // }
  // // if (getRightMotorPosition() < 0 && rightHangMotorSpeed < 0) {
  // // rightHangMotorSpeed = 0;
  // // // }

  // // if (getMotorPosition() > HangConstants.kHangMotorMaxExtension &&
  // hangMotorSpeed > 0) {
  // // hangMotorSpeed = 0;
  // }
}

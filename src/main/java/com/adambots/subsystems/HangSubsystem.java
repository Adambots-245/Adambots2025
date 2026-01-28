// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.io.Console;
import java.io.PrintStream;

import com.adambots.actuators.BaseMotor;
import com.adambots.actuators.BaseServo;
import com.adambots.actuators.BaseSolenoid;
import com.adambots.sensors.LimitSwitch;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  BaseMotor hangMotor;
  BaseSolenoid hangSolenoid;
  BaseServo hangServo;
  double hangMotorSpeed = 0;
  LimitSwitch hangLimitSwitch;

  public HangSubsystem(BaseMotor hangMotor, BaseSolenoid hangSolenoid, BaseServo hangServo, LimitSwitch hangLimitSwitch) {
   
    this.hangMotor = hangMotor;
    this.hangSolenoid = hangSolenoid;
    this.hangServo = hangServo;
    this.hangLimitSwitch = hangLimitSwitch;
    hangMotor.setInverted(true);
    hangMotor.setBrakeMode(true);
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

  public void toggleSolenoids() {
    hangSolenoid.toggle();
  }

  public boolean getSolenoid() {
    return hangSolenoid.get();
  }

  public void releaseServo() {
    hangServo.setAngle(50);
  }

  public void returnServo() {
    hangServo.setAngle(130);
  }

  public boolean isLimitPressed(){
    return hangLimitSwitch.isDetecting();
  }
  // public double getPitch(){
  //   return gyro.getPitch();
  // }

  // public double getYaw(){
  //   return gyro.getContinuousYawDeg();
  // }
  
  // public double getRoll(){
  //   return gyro.getRoll();
  // }

  @Override
  public void periodic() {

    // failSafes();
    hangMotor.set(hangMotorSpeed);
    SmartDashboard.putBoolean("Hang/Solenoid", hangSolenoid.get());
    
    SmartDashboard.putBoolean("Hang/Limit Switch", hangLimitSwitch.isDetecting());
    
    // SmartDashboard.putNumber("Pitch", getPitch());
    // SmartDashboard.putNumber("Roll", getRoll());
    // SmartDashboard.putNumber("Yaw", getYaw());
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

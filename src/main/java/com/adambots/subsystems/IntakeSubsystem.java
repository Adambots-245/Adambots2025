// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.IntakeConstants;
import com.adambots.actuators.BaseMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private BaseMotor intakeMotor;
  private double motorSpeed;
  
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(BaseMotor intakeMotor) {
      this.intakeMotor = intakeMotor;
  }

  public void intake() {
    System.out.println("Calling Intake");
    motorSpeed = IntakeConstants.kMaxSpeed;
  }

  public void stopIntake() {
    motorSpeed = 0;
  }

  public void reverseIntake() {
    motorSpeed = IntakeConstants.kReverseSpeed;
  }

  public void slowIntake() {
    motorSpeed = IntakeConstants.kLowSpeed;
  }

  @Override
  public void periodic() {
    intakeMotor.set(motorSpeed);
  }
}

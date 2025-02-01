// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.IntakeConstants;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.LimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private BaseMotor intakeMotor;
  private double motorSpeed;
  private LimitSwitch firstIntakeLimit;
  private LimitSwitch secondIntakeLimit;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(BaseMotor intakeMotor, LimitSwitch firstIntakeLimit, LimitSwitch secondIntakeLimit) {
      this.firstIntakeLimit = firstIntakeLimit;
      this.secondIntakeLimit = secondIntakeLimit;
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

  public boolean isFirstLimitDetecting() {
    return firstIntakeLimit.isDetecting();
  }
  
  public boolean isSecondLimitDetecting() {
    return secondIntakeLimit.isDetecting();
  }
  
  @Override
  public void periodic() {
    intakeMotor.set(motorSpeed);
  }
}

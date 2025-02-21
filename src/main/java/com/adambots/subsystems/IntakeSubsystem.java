// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.IntakeConstants;
import com.adambots.actuators.BaseActuator;
import com.adambots.sensors.BaseDistanceSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // private BaseMotor intakeMotor;
  private BaseActuator intakeActuator;
  private double motorSpeed;
  private BaseDistanceSensor CANrange;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(BaseActuator intakeActuator, BaseDistanceSensor CANrange) {
      this.CANrange = CANrange;
      this.intakeActuator = intakeActuator;
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

  public boolean isDetecting() {
    return CANrange.getDistanceInCentimeters() < IntakeConstants.kDistanceToDetect;
  }
  
  @Override
  public void periodic() {
    intakeActuator.set(motorSpeed);
  }
}

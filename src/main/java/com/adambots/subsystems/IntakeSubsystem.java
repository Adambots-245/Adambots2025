// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.IntakeConstants;
import com.adambots.actuators.BaseActuator;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseDistanceSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // private BaseMotor intakeMotor;
  private BaseMotor minionMotor;
  private double coralIntakeSpeed = 0.0;
  private double algaeIntakeSpeed = 0.0;

  private int counter = 0;
  private BaseDistanceSensor CANrange;
  private BaseActuator algaeGripper;
  private BaseActuator algaeRunner;

  private PIDController intakePID = new PIDController(0.03, 0, 0.001);
  private double goalVelocity = 0;


  /**
   * Constructor for IntakeSubsystem - used to intake coral and algae
   * @param topCoralActuator - top actuator for coral intake
   * @param bottomCoralActuator - bottom actuator for coral intake; if we are using only one, pass null to this
   * @param algaeGripper - actuator for algae gripper - a servo set to angular (SERVO) mode
   * @param algaeRunner - actuator for algae runner - a motor set to CR mode
   * @param CANrange - distance sensor for detecting coral
   */
  public IntakeSubsystem(BaseMotor minionMotor,
      BaseActuator algaeGripper, BaseActuator algaeRunner, BaseDistanceSensor CANrange) {

    this.CANrange = CANrange;
    this.minionMotor = minionMotor;
    minionMotor.setBrakeMode(true);

    this.algaeGripper = algaeGripper;
    this.algaeRunner = algaeRunner;
  }

  public void intakeCoral() {
    System.out.println("Calling Intake");
    coralIntakeSpeed = IntakeConstants.kMaxSpeed;

    goalVelocity = 8.25;
  }

  public void intakeCoral(double speed) {
    System.out.println("Calling Intake");
    coralIntakeSpeed = speed;

    goalVelocity = 19;
  }


  public void stopCoralIntake() {
    coralIntakeSpeed = 0;
    goalVelocity = 0;
  }

  public void reverseCoralIntake() {
    coralIntakeSpeed = IntakeConstants.kReverseSpeed;
  }

  public void slowCoralIntake() {
    coralIntakeSpeed = IntakeConstants.kLowSpeed;
  }

  public boolean isDetectingCoral() {
    return CANrange.getDistanceInCentimeters() < IntakeConstants.kDistanceToDetect;
  }

  public void intakeAlgae() {
    algaeIntakeSpeed = -IntakeConstants.kMaxSpeed;
    counter = 0;
  }

  public void stopAlgaeIntake() {
    algaeIntakeSpeed = 0;
  }

  public void reverseAlgaeIntake() {
    algaeIntakeSpeed = -IntakeConstants.kReverseSpeed;
  } 

  @Override
  public void periodic() {
    double intakeSpeed = intakePID.calculate(minionMotor.getVelocity(), goalVelocity);

    if (goalVelocity == 0) {
      minionMotor.set(0);
    } else {
      minionMotor.set(intakeSpeed);
    }

    // minionMotor.set(-coralIntakeSpeed); // run CW to intake coral

    // if (bottomCoralActuator != null) {
    //   bottomCoralActuator.set(coralIntakeSpeed); // run CCW to intake coral
    // }
    

    SmartDashboard.putBoolean("Intake/CANrange", isDetectingCoral());
    SmartDashboard.putNumber("Intake/Minion Speed", minionMotor.getVelocity());



    // Algae intake logic - there two servos, one to grip the Algae and one to run the Algae into the intake.
    // the gripper is running in servo mode, so it will hold the algae in place until the runner is ready to intake it.
    // the runner is running in CR mode, so it will run the algae into the intake. However, it won't be able to keep gripping it.
    // Hence, wait for x number of secods (pulse) before stopping the runner and then restarting it. If you don't do this, the servo will stop the runner to prevent brownouts.
    // Do this only for positive speeds. If you want to reverse the intake, don't do this.
    if (algaeIntakeSpeed > 0) { //intake the algae

      // This loop will run every 20 ms. So, convert the seconds to milliseconds and divide by 20 to get the number of loops to run.
      if (counter >= (IntakeConstants.kAlgaeIntakePulseSeconds * 1000/20)) {
        algaeGripper.set(algaeIntakeSpeed);
        algaeRunner.set(0);
        algaeGripper.set(0);
        counter = 0;
      } else {
        algaeRunner.set(algaeIntakeSpeed);
        algaeGripper.set(-algaeIntakeSpeed);

        counter++;
      }

    } else { // Reverse or stop the algae intake
      algaeRunner.set(algaeIntakeSpeed);
      algaeGripper.set(-algaeIntakeSpeed);
    }
  }
}

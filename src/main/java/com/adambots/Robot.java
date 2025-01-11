/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.PDM.setSwitchableChannel(true);
    RobotMap.gyro.resetYaw();

    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // newly-scheduled commands, running already-scheduled commands, removing finished or
    // interrupted commands, and running subsystem periodic() methods. This must be called from the
    // robot's periodic block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (Constants.enableAutomaticShuffleboardRecording) {
      Shuffleboard.stopRecording();
      System.out.println("Shuffleboard recording ended");
    }
  }

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll(); //Cancel all teleop or lingering commands

    if (Constants.enableAutomaticShuffleboardRecording) {
      Shuffleboard.startRecording();
      System.out.println("Autonomous Shuffleboard recording started - " + DriverStation.getMatchTime());
    }

    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("Auton Selected: " + m_autonomousCommand.toString());

    System.out.println("Init Auton.........");

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  //@Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (Constants.enableAutomaticShuffleboardRecording) {
      Shuffleboard.startRecording();
      System.out.println("Teleop Shuffleboard recording started - " + DriverStation.getMatchTime());
    }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

}

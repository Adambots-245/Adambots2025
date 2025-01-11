// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.io.IOException;
import java.util.HashMap;

import org.json.simple.parser.ParseException;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.VisionConstants;
import com.adambots.RobotMap;
import com.adambots.sensors.BaseGyro;
import com.adambots.utils.ModuleMap;
import com.adambots.utils.Utils;
import com.adambots.vision.LimelightHelpers;
import com.adambots.vision.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  private RobotConfig config;
  // The gyro sensor
  private final BaseGyro m_gyro;

  // Pose Estimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Boolean frontLimelightFlag = false;

  // Odometry class for tracking robot pose
  private HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;
    frontLimelightFlag = false;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }

    System.out.println(this.getName() + "Initializing");

    
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPTranslationController, AutoConstants.kITranslationController, AutoConstants.kDTranslationController), // Translation PID constants
            new PIDConstants(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> Utils.isOnRedAlliance(), // Flips path if on the red side of the field
        this // Reference to this subsystem to set requirements
    );

    // AutoBuilder.configureHolonomic(
    // this::getPose, // Robot pose supplier
    // this::resetOdometry, // Method to reset odometry (will be called if your auto
    // has a starting pose)
    // this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    // this::setChassisSpeeds, // Method that will drive the robot given ROBOT
    // RELATIVE ChassisSpeeds
    // new HolonomicPathFollowerConfig(
    // new PIDConstants(AutoConstants.kPTranslationController, 0,
    // AutoConstants.kDTranslationController),
    // new PIDConstants(AutoConstants.kPThetaController, 0,
    // AutoConstants.kDThetaController),
    // DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
    // DriveConstants.kDrivebaseRadius, // Drive base radius in meters. Distance
    // from robot center to furthest module
    // new ReplanningConfig(false, false) // Default path replanning config. See the
    // API for the options here
    // ),
    // () -> Robot.isOnRedAlliance(), //Flips path if on the red side of the field
    // this // Reference to this subsystem to set requirements
    // );

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getContinuousYawRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules),
        new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.9, 0.9, 6));
  }

  /**
   * Returns the shortest error between two angle measurements in radians, not
   * matter whether angles are discrete or continuous.
   *
   * @param setpoint    The first angle measurement in radians.
   * @param measurement The second angle measurement in radians.
   */
  public double getContinuousAngleError(double setpoint, double measurement) {
    return MathUtil.inputModulus(setpoint - measurement, -Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the position of the robot on the ShuffleBoard field
    // Constants.odomField.setRobotPose(getPose());
    // System.out.println(this.getName() + ".periodic()");

    // m_poseEstimator.updateWithTime(System.currentTimeMillis()/1000,
    // m_gyro.getContinuousYawRotation2d(),
    // ModuleMap.orderedModulePositions(swerveModules));
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_gyro.getContinuousYawRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules));

    // Update the odometry in the periodic block
    Pose2d visionPose = null;
    if (!DriverStation.isAutonomous()) {
      if (frontLimelightFlag && VisionHelpers.isDetected(VisionConstants.defaultAprilLimelite)) {
        visionPose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.defaultAprilLimelite); // Fixed code
        // if (visionPose.getY() > 1 &&
        // getContinuousAngleError(visionPose.getRotation().getRadians(),
        // m_gyro.getContinuousYawRad()) < Math.toRadians(20) &&
        // VisionHelpers.getAprilHorizDist(VisionConstants.defaultAprilLimelite) < 4.5)
        // {
        // m_poseEstimator.addVisionMeasurement(visionPose,
        // System.currentTimeMillis()/1000);

        // }
        m_poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
      }
      // else if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      // Pose2d visionPose =
      // VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite);
      // if (visionPose.getY() > 1 &&
      // getContinuousAngleError(visionPose.getRotation().getRadians(),
      // m_gyro.getContinuousYawRad()) < Math.toRadians(20) &&
      // VisionHelpers.getAprilHorizDist(VisionConstants.aprilLimelite) < 4.5) {
      // m_poseEstimator.addVisionMeasurement(visionPose,
      // System.currentTimeMillis()/1000);
      // }
      // }

      Constants.odomField.setRobotPose(getPose());
      if (visionPose != null) {
        // Constants.rearLLField.setRobotPose(visionPose);
      }
    }

    // Constants.frontLLField.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite));
    // Constants.rearLLField.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite));
  }

  /**
   * Sets whether or not we should upate odom using the limelight on the front of
   * the robot.
   *
   * @param flagState set the flag true or false.
   */
  public void setArmLimelightFlag(Boolean flagState) {
    frontLimelightFlag = flagState;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry and gyro to the specified pose, including x, y, and
   * heading.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    RobotMap.gyro.resetYawToAngle(pose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(pose.getRotation(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed (m/s) of the robot in the x direction (forward).
   * @param ySpeed        Speed (m/s) of the robot in the y direction (sideways).
   * 
   * @param rot           Angular rate of the robot.
   * 
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      if (Utils.isOnRedAlliance()) {
        setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, m_gyro.getContinuousYawRotation2d()));
      } else {
        setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getContinuousYawRotation2d()));
      }
    } else {
      setChassisSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
  }

  /**
   * Stops the drivetrain
   */
  public void stop() {
    ModuleMap.stopModules(swerveModules);
  }

  /**
   * Sets the swerve module states as according to the chassis speeds requested
   *
   * @param chassisSpeeds The desired ChassisSpeeds of the robot
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, desiredStates);
  }

  /**
   * Gets the chassis speeds of the robot as calculated from the swerve module
   * states
   *
   * @return The ChassisSpeeds of the robot
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(ModuleMap.orderedModuleStates(swerveModules));
  }
}
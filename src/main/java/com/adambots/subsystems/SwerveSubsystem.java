// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.io.File;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meter;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.ModuleConstants;
import com.adambots.utils.Utils;
import com.adambots.vision.PhotonVision;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final boolean visionDriveTest = false;
  private PhotonVision vision;

  /**
   * Creates a new SwerveSubsystem. Adapted from YAGSL-Example
   * Talk to Mr.B before making major changes to this file.
   */
  public SwerveSubsystem(File directory) {

    // The 2 value below will be defined in the JSON configuration file. However,
    // alternatively, we can do it here.
    // Use the same values when defining the JSON file

    // The angleConversionFactor is used to convert the steering encoder readings
    // to actual wheel angles. This is necessary for accurate control of the swerve
    // drive system, ensuring that the wheels are oriented correctly based on the
    // desired direction and rotation.
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ModuleConstants.kSteeringGearRatio);

    // Calculates the drive conversion factor in meters per rotation for the swerve
    // module.
    // This is required to convert the drive motor encoder readings to actual wheel
    // speeds.
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to
    // meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(ModuleConstants.kWheelRadiusMeters,
        1 / ModuleConstants.kSwerveModuleFinalGearRatio);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    // Adjust this value to control the amount of telemetry data that is printed to
    // the console. Turn it off or to low or pose for competition.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      // Loads the conversion factors via JSON files
      swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      // Code is broken - stop the robot
      throw new RuntimeException(e);
    }

    // Heading correction uses the heading PID from controllerproperties.json and
    // the current yaw to calculate an omega (angular velocity) turning speed using
    // SwerveController.headingCalculate
    // Calculating Omega (ω): The SwerveController.headingCalculate function
    // computes the necessary angular velocity (ω) to correct any deviation from the
    // desired heading. This calculation determines how quickly and in which
    // direction the robot should rotate to achieve the target orientation.
    // Applying the Correction: The computed ω is then used to adjust the swerve
    // modules' wheel angles and speeds, facilitating the desired rotational
    // movement.
    swerveDrive.setHeadingCorrection(false);

    // Cosine compensation is a technique used in swerve drive systems to enhance
    // control and efficiency by adjusting the speed of each wheel based on its
    // alignment with the desired movement direction.
    // In a swerve drive, each wheel can rotate independently to achieve precise
    // movement. However, when a wheel's orientation doesn't perfectly match the
    // intended direction, it can introduce inefficiencies or unintended forces.
    // To address this, cosine compensation scales the wheel's speed by the cosine
    // of the angle difference between its current orientation and the desired
    // direction.
    // This may cause unintended consequences. Hence, test it before fully enabling
    // it.
    // Will not work in simulation
    swerveDrive.setCosineCompensator(false);

    // Angular Velocity Compensation is a feature designed to mitigate the skewing
    // effect that can occur when a swerve-drive robot moves linearly while
    // simultaneously rotating.
    // This skewing results from the combined translational and rotational
    // movements, causing the robot to deviate from its intended path.
    // The primary goal of angular velocity compensation is to adjust the wheel
    // velocities in real-time to counteract the undesired lateral movements induced
    // by rotation.
    // By doing so, the robot maintains a straighter trajectory during combined
    // translational and rotational motions, enhancing overall maneuverability and
    // control.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

    // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // YAGSL will automatically synchronize the absolute encoders and internal
    // encoders of the angle/steering/azimuth motor controllers everytime the module
    // is at rest for half-a-second if the delta between the internal encoder and
    // absolute encoder is greater than the deadband.
    // Enable this after initial setup and tests; set the deadband in degrees as the
    // second parameter.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1.0);

    // Offset Offloading is where the absolute encoder offset is stored on the
    // absolute encoder (or Motor Controller if the absolute encoder is attached to
    // the motor controller) instead of the roboRIO. This normally results in a
    // faster loop cycle on the roboRIO however it can add some instability to the
    // Swerve Drive if anything breaks during a match, like a CAN bus or a low
    // brown-out.
    swerveDrive.pushOffsetsToEncoders();

    if (visionDriveTest) {
      setupPhotonVision();

      // Stop the odometry thread if we are using vision that way we can synchronize
      // updates better.
      swerveDrive.stopOdometryThread();
    }

    setupPathPlanner();
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision() {
    vision = new PhotonVision(swerveDrive::getPose, swerveDrive.field);
  }

  private void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return Utils.isOnRedAlliance();
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  @Override
  public void simulationPeriodic() {

  }

  /**
   * Get the distance to a specific AprilTag.
   *
   * @param tagID The ID of the AprilTag to get the distance to.
   * @return Distance to a specific AprilTag in meters.
   */
  public double getDistanceToAprilTag(int tagID) {

    // Taken from PhotonUtils.getDistanceToPose
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
  }

  /**
   * Get the yaw to aim at an AprilTag.
   * 
   * @param tagID The ID of the AprilTag to aim at.
   * @return {@link Rotation2d} of which you need to achieve.
   */
  public Rotation2d getAprilTagYaw(int tagID) {
    // Taken from PhotonUtils.getYawToPose()
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    Translation2d relativeTrl = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  /**
   * Retrieves the SwerveDrive instance from the subsystem. Use cautiously.
   *
   * @return the SwerveDrive instance
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

 

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        DriveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (Utils.isOnRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Checks if the vision drive test mode is enabled.
   *
   * @return true if vision drive test mode is enabled, false otherwise.
   */
  public boolean isVisionDriveTest() {
    return visionDriveTest;
  }

  /**
   * Get the PhotonVision instance.
   *
   * @return The PhotonVision instance.
   */
  public PhotonVision getVision() {
    return vision;
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}

package com.adambots;

import java.io.File;

import com.adambots.Constants.DriveConstants;
import com.adambots.commands.driveCommands.DriveCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.utils.Dash;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import swervelib.SwerveInputStream;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSubsystem drivetrainSubsystem = new
  // DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.gyro);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
  // private final CANdleSubsystem candleSubsytem = new
  // CANdleSubsystem(RobotMap.candleLEDs);

  private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);

  // Creates a SmartDashboard element to allow drivers to select differnt autons
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure commands to run periodically during robot operation
    setupDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Register commands for use in PathPlanner
    registerNamedCommands();

    // configure the dashboard
    setupDashboard();
  }

  public void teleopInit() {
    if (DriverStation.isFMSAttached()) {
      // Initialize Subsystems here
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   */
  private void configureButtonBindings() {
    // JOYSTICK BINDINGS SHOULD BE IN NUMERICAL ORDER TO PREVENT DOUBLE BINDINGS

    if (Robot.isSimulation()) {
      Buttons.XboxStartButton
          .onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    if (DriverStation.isTest()) {
      Buttons.XboxBButton.whileTrue(driveCommands.sysIdDriveMotorCommand());
      Buttons.XboxXButton.whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
      Buttons.XboxYButton.whileTrue(driveCommands.driveToDistanceCommand(1.0, 0.2));
      Buttons.XboxStartButton.onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
      Buttons.XboxBackButton.whileTrue(driveCommands.centerModulesCommand());
      Buttons.XboxLeftBumper.onTrue(Commands.none());
      Buttons.XboxRightBumper.onTrue(Commands.none());
    } else {
      Buttons.XboxAButton.onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
      Buttons.XboxXButton.onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
      Buttons.XboxBButton.whileTrue(
          driveCommands.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      Buttons.XboxYButton.whileTrue(driveCommands.aimAtAprilTag(2, 1));
      Buttons.XboxStartButton.whileTrue(Commands.none());
      Buttons.XboxBackButton.whileTrue(Commands.none());
      Buttons.XboxLeftBumper.whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
      Buttons.XboxRightBumper.onTrue(Commands.none());
    }

  }

  /**
   * Register named commands for use in PathPlanner
   */
  private void registerNamedCommands() {
    // NamedCommands.registerCommand("ShootPreload", new SequentialCommandGroup(
    // new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem,
    // candleSubsytem, ShooterConstants.mediumSpeed, ArmConstants.speakerState),
    // new WaitCommand(1),
    // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem),
    // new InstantCommand(() ->
    // armSubsystem.setCurrentState(ArmConstants.closeFloorShootState))
    // ));
  }

  /**
   * Configure the dashboard with useful data for driving and debugging
   */
  private void setupDashboard() {
    autoChooser = AutoBuilder.buildAutoChooser();

    // Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);

    // SmartDashboard.putData("FrontLL Field", Constants.frontLLField);
    // SmartDashboard.putData("RearLL Field", Constants.rearLLField);
    SmartDashboard.putData("Odom Field", Constants.odomField);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);

    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    // Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    // Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());

    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());
    Dash.add("pitch", () -> RobotMap.gyro.getPitch());
    Dash.add("roll", () -> RobotMap.gyro.getRoll());
  }

  private void setupDefaultCommands() {
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        Buttons.forwardSupplier,
        Buttons.sidewaysSupplier)
        .withControllerRotationAxis(Buttons.rotateSupplier)
        .deadband(DriveConstants.kDeadZone)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(Buttons.sidewaysSupplier, Buttons.forwardSupplier)
        .headingWhile(true);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = driveCommands.driveFieldOriented(driveDirectAngle);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = driveCommands.driveFieldOriented(driveAngularVelocity);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        Buttons.forwardSupplier,
        Buttons.sidewaysSupplier)
        .withControllerRotationAxis(Buttons.rotateSupplier)
        .deadband(DriveConstants.kDeadZone)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
        .withControllerHeadingAxis(() -> Math.sin(
            Buttons.rotateSupplier.getAsDouble() * Math.PI)
            * (Math.PI * 2),
            () -> Math.cos(
                Buttons.rotateSupplier.getAsDouble() * Math.PI)
                *
                (Math.PI * 2))
        .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = driveCommands.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAngularVelocitySim = driveCommands.driveFieldOriented(driveAngularVelocitySim);

    swerveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAngularVelocitySim);

    if (DriverStation.isTest()) {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    }

    // drivetrainSubsystem.setDefaultCommand(
    // new RunCommand(
    // () -> drivetrainSubsystem.drive(
    // Buttons.forwardSupplier.getAsDouble() *
    // DriveConstants.kMaxSpeedMetersPerSecond,
    // Buttons.sidewaysSupplier.getAsDouble() *
    // DriveConstants.kMaxSpeedMetersPerSecond,
    // Buttons.rotateSupplier.getAsDouble() * DriveConstants.kTeleopRotationalSpeed,
    // true),
    // drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return driveCommands.getAutonomousCommand("New Auto");
  }
}
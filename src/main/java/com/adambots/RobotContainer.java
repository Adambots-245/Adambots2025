package com.adambots;

import java.io.File;

import com.adambots.Constants.DriveConstants;
import com.adambots.commands.driveCommands.DriveCommands;
import com.adambots.commands.elevatorCommands.ElevatorCommands;
import com.adambots.commands.intakeCommands.IntakeCommands;
import com.adambots.commands.scoringCommands.ScoringCommands;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.ElevatorSubsystem.ElevatorState;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.subsystems.WristSubsystem;
import com.adambots.subsystems.WristSubsystem.WristState;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
  private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs);
  IntakeSubsystem intakesubsystem = new IntakeSubsystem(RobotMap.topCoralActuator, RobotMap.bottomCoralActuator, RobotMap.algaeGripper, RobotMap.algaeRunner, RobotMap.CANrange);
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotMap.elevatorMotor);
  WristSubsystem wristSubsystem = new WristSubsystem(RobotMap.wristMotor, RobotMap.encoder);

  // Add commands here
  private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);
  private final IntakeCommands intakeCommands = new IntakeCommands(intakesubsystem);
  private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem, wristSubsystem);
  private final ScoringCommands scoringCommands = new ScoringCommands(intakesubsystem);

  // Creates a SmartDashboard element to allow drivers to select differnt autons
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private static SwerveInputStream driveAngularVelocity;

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
      Buttons.XboxBackButton.whileTrue(driveCommands.centerModulesCommand());
      Buttons.XboxLeftBumper.onTrue(Commands.none());
      // RobotMap.gyro.resetYaw();
      // Buttons.JoystickButton6.onTrue(new InstantCommand(RobotMap.gyro.resetYaw()));
      Buttons.XboxRightBumper.onTrue(Commands.none());
    } else {
      Buttons.JoystickButton7.onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));

      Buttons.XboxXButton.onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
      Buttons.XboxBButton.whileTrue(
          driveCommands.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      Buttons.XboxYButton.whileTrue(driveCommands.aimAtAprilTag(2, 1));
      Buttons.XboxStartButton.whileTrue(Commands.none());
      Buttons.XboxBackButton.whileTrue(Commands.none());
      Buttons.XboxLeftBumper.whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
      Buttons.XboxRightBumper.onTrue(Commands.none());
      
      Buttons.JoystickButton1.onTrue(scoringCommands.scoreAlgae());
      Buttons.JoystickButton2.onTrue(scoringCommands.stopScoringAlgae());
      Buttons.JoystickButton3.onTrue(scoringCommands.scoreCoral());
      Buttons.JoystickButton4.onTrue(scoringCommands.stopScoringCoral());
      Buttons.JoystickButton5.onTrue(elevatorCommands.moveElevatorToStateCommand(ElevatorState.L4));
      Buttons.JoystickButton6.onTrue(elevatorCommands.moveElevatorToStateCommand(ElevatorState.INTAKE));
      Buttons.JoystickButton8.onTrue(elevatorCommands.moveElevatorToStateCommand(ElevatorState.L1));
      Buttons.JoystickButton9.onTrue(elevatorCommands.moveElevatorToStateCommand(ElevatorState.L2));
      Buttons.JoystickButton10.onTrue(elevatorCommands.moveWristToStateCommand(WristState.L1));
      Buttons.JoystickButton11.onTrue(elevatorCommands.moveWristToStateCommand(WristState.L2));
      Buttons.JoystickButton12.onTrue(elevatorCommands.moveWristToStowedCommand());
      Buttons.JoystickButton13.onTrue(elevatorCommands.moveWristToIntakeCommand());
      Buttons.JoystickButton14.onTrue(elevatorCommands.moveWristToStateCommand(WristState.L1));
      Buttons.JoystickButton15.onTrue(elevatorCommands.moveWristToStateCommand(WristState.L2));
      Buttons.JoystickButton16.onTrue(elevatorCommands.moveWristToStateCommand(WristState.L3));

      Buttons.XboxAButton.onTrue(elevatorCommands.moveToL1Command());
      Buttons.XboxBButton.onTrue(elevatorCommands.moveToL2Command());
      Buttons.XboxXButton.onTrue(elevatorCommands.moveToL3Command());
      Buttons.XboxYButton.onTrue(elevatorCommands.moveToL4Command());
      Buttons.XboxLeftStickButton.onTrue(intakeCommands.intakeAlgae());
      Buttons.XboxRightStickButton.onTrue(intakeCommands.stopIntakeAlgae());
      Buttons.XboxStartButton.onTrue(intakeCommands.reverseIntakeAlgae());
      Buttons.XboxBackButton.onTrue(intakeCommands.reverseIntakeCoral());

      Buttons.XboxDPadN.whileTrue(elevatorCommands.moveElevatorUp());
      Buttons.XboxDPadS.whileTrue(elevatorCommands.moveElevatorDown());
      Buttons.XboxDPadE.whileTrue(elevatorCommands.moveWristUp());
      Buttons.XboxDPadW.whileTrue(elevatorCommands.moveWristDown());

    SmartDashboard.putData("Intake Coral", intakeCommands.intakeCoral());
    SmartDashboard.putData("Stop Intake Coral", intakeCommands.stopIntakeCoral());
    SmartDashboard.putData("Slow Intake Coral", intakeCommands.slowIntakeCoral());
    SmartDashboard.putData("Reverse Intake Coral", intakeCommands.reverseIntakeCoral());
    SmartDashboard.putData("Score Coral", scoringCommands.scoreCoral());
    SmartDashboard.putData("Stop Score Coral", scoringCommands.stopScoringCoral());
    SmartDashboard.putData("Stop Score Algae", scoringCommands.stopScoringAlgae());

    SmartDashboard.putData("Intake State", elevatorCommands.moveToIntakeCommand());
    SmartDashboard.putData("L1 State", elevatorCommands.moveToL1Command());
    SmartDashboard.putData("L2 State", elevatorCommands.moveToL2Command());
    SmartDashboard.putData("L3 State", elevatorCommands.moveToL3Command());
    SmartDashboard.putData("L4 State", elevatorCommands.moveToL4Command());

    SmartDashboard.putData("Wrist Stowed", elevatorCommands.moveWristToIntakeCommand());
    SmartDashboard.putData("Wrist Intake", elevatorCommands.moveWristToStowedCommand());
    SmartDashboard.putData("Wrist L1", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L1));
    SmartDashboard.putData("Wrist L2", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L2));
    SmartDashboard.putData("Wrist L3", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L3));
    SmartDashboard.putData("Wrist L4", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L4));
    SmartDashboard.putData("Wrist Down", elevatorCommands.moveWristDown());
    SmartDashboard.putData("Wrist Up", elevatorCommands.moveWristUp());
    
    SmartDashboard.putData("Elevator L1", elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L1));
    SmartDashboard.putData("Elevator L2", elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L2));
    SmartDashboard.putData("Elevator L3", elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L3));
    SmartDashboard.putData("Elevator L4", elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L4));
    SmartDashboard.putData("Elevator Up", elevatorCommands.moveElevatorUp());
    SmartDashboard.putData("Elevator Down", elevatorCommands.moveElevatorDown());

    SmartDashboard.putData("Intake Algae", intakeCommands.intakeAlgae());
    SmartDashboard.putData("Stop Algae", intakeCommands.stopIntakeAlgae());
    SmartDashboard.putData("Reverse Algae", intakeCommands.reverseIntakeAlgae());
    }

    // swerveSubsystem.getVision().getTargetFromId(1, PhotonVision.Cameras.CENTER_CAM);
    // PhotonVision.getAprilTagPose(1, new Transform2d(new Translation2d(2.0, 2.0), new Rotation2d()));
    // swerveSubsystem.getVision().getDistanceFromAprilTag(1);

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
    Dash.add("CANrange Dist", ()-> RobotMap.CANrange.getDistanceInInches());
    // Dash.add("ServoHub", ()->RobotMap.hub.getDeviceVoltage());
    // Dash.add("Wrist Encoder", ()->RobotMap.encoder.getAbsolutePositionDegrees());

    // SmartDashboard.putData("FrontLL Field", Constants.frontLLField);
    // SmartDashboard.putData("RearLL Field", Constants.rearLLField);
    // SmartDashboard.putData("Odom Field", Constants.odomField);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);

    // Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    // Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    // Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());

    // Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());
    // Dash.add("pitch", () -> RobotMap.gyro.getPitch());
    // Dash.add("roll", () -> RobotMap.gyro.getRoll());
  }

  private void setupDefaultCommands() {
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
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
    // Creates a new SwerveInputStream for driving with direct angle simulation.
    // 
    // The controller heading axis is calculated using the sine and cosine of the 
    // rotation supplier's value multiplied by π (Math.PI), and then scaled by 2π (2 * Math.PI).
    // 
    // The sine function is used to calculate the x-axis component of the heading, 
    // while the cosine function is used to calculate the y-axis component of the heading.
    // 
    // The headingWhile method is called with a true value to maintain the heading.
    // 
    // @return A new SwerveInputStream with the specified controller heading axis and heading behavior.
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
  }

  public static SwerveInputStream getDriveAngularVelocity() {
    return driveAngularVelocity;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return driveCommands.getAutonomousCommand("New Auto");
  }
}
package com.adambots;

import java.io.File;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.ElevatorConstants;
import com.adambots.commands.HangCommands;
import com.adambots.lib.commands.driveCommands.DriveCommands;
import com.adambots.commands.driveCommands.DriveToLocationAdvanced;
import com.adambots.commands.driveCommands.RotateToAngleCommand;
import com.adambots.commands.driveCommands.DriveToLocationAdvanced.AlignLocation;
import com.adambots.commands.elevatorCommands.ElevatorCommands;
import com.adambots.commands.intakeCommands.IntakeCommands;
import com.adambots.commands.scoringCommands.ScoringCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.HangSubsystem;
import com.adambots.subsystems.ElevatorSubsystem.ElevatorState;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.subsystems.WristSubsystem;
import com.adambots.subsystems.WristSubsystem.WristState;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Dash;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    IntakeSubsystem intakesubsystem = new IntakeSubsystem(RobotMap.minionMotor,
            RobotMap.algaeLeftMotor, RobotMap.algaeRightMotor, RobotMap.coralCANrange, RobotMap.algaeCANrange);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotMap.elevatorMotor, RobotMap.lowerElevatorLimitSwitch, RobotMap.upperElevatorLimitSwitch);
    WristSubsystem wristSubsystem = new WristSubsystem(RobotMap.wristMotor, RobotMap.wristEncoder);
    HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.climbMotor, RobotMap.climbSolenoid, RobotMap.climbServo,
            RobotMap.hangLimitSwitch);
    // Add commands here
    private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);
    private final IntakeCommands intakeCommands = new IntakeCommands(intakesubsystem, candleSubsytem);
    private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem, wristSubsystem,
            intakeCommands, intakesubsystem);
    private final ScoringCommands scoringCommands = new ScoringCommands(intakesubsystem, elevatorSubsystem, candleSubsytem);
    private final HangCommands hangCommands = new HangCommands(hangSubsystem, candleSubsytem);

    // Creates a SmartDashboard element to allow drivers to select differnt autons
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static SwerveInputStream driveAngularVelocity;

    private Integer aprilTagId = 6;

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
            // DONT USE THIS IF NOT NEEDED
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

        // if (Robot.isSimulation()) {
        // // REEF ID
        // Buttons.JoystickButton1.onTrue(new InstantCommand(() -> aprilTagId = 17));
        // Buttons.JoystickButton2.onTrue(new InstantCommand(() -> aprilTagId = 5));

        // // BARGE ID
        // // Buttons.JoystickButton1.onTrue(new InstantCommand(() -> aprilTagId = 14));
        // // Buttons.JoystickButton2.onTrue(new InstantCommand(() -> aprilTagId = 5));

        // // ALIGN TO HUMAN PLAYER TEST
        // // Buttons.JoystickButton3
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.HUMAN_PLAYER_LEFT, candleSubsytem));
        // // Buttons.JoystickButton4
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.HUMAN_PLAYER_RIGHT, candleSubsytem));

        // // ALIGN TO REEF TEST
        // // Buttons.JoystickButton2
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.MIDDLE_ALGAE,
        // // candleSubsytem));
        // Buttons.JoystickButton3
        // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // AlignLocation.LEFT_POLE,
        // candleSubsytem));
        // Buttons.JoystickButton4
        // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // AlignLocation.RIGHT_POLE,
        // candleSubsytem));

        // // ANGLE TO REEF TEST
        // // Buttons.JoystickButton4
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.REEF_ANGLE,
        // // candleSubsytem));

        // // ALIGN TO BARGE TEST
        // // Buttons.JoystickButton2
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.BARGE_LEFT,
        // // candleSubsytem));
        // // Buttons.JoystickButton3
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.BARGE_MIDDLE,
        // // candleSubsytem));
        // // Buttons.JoystickButton4
        // // .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId,
        // // AlignLocation.BARGE_RIGHT,
        // // candleSubsytem));

        // // Buttons.JoystickButton4
        // // .whileTrue(driveCommands.getDistanceFromAprilTag(17));

        // } else {
        Buttons.JoystickButton1.onTrue(scoringCommands.scoreAlgaeDynamic().andThen(scoringCommands.scoreCoral()));
        Buttons.JoystickButton1.onFalse(scoringCommands.stopScoringCoral());
        Buttons.JoystickButton1.onFalse(scoringCommands.stopScoringAlgae());

        // Buttons.JoystickButton9.onTrue(scoringCommands.scoreAlgaeDynamic());
        // Buttons.JoystickButton9.onFalse(scoringCommands.stopScoringAlgae());

        Buttons.JoystickButton2
                .whileTrue(new RotateToAngleCommand(swerveSubsystem, 90, false));

        // Buttons.JoystickButton3
        //         .whileTrue(
        //                 new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.HUMAN_PLAYER_LEFT,
        //                         candleSubsytem));
        // Buttons.JoystickButton2
        //         .whileTrue(
        //                 new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.HUMAN_PLAYER_RIGHT,
        //                         candleSubsytem, driveCommands));

        // Buttons.JoystickButton2.whileTrue(new RotateToAngleCommand(swerveSubsystem, 90));
        Buttons.JoystickButton3.whileTrue(new RotateToAngleCommand(swerveSubsystem,
        125, true));
        Buttons.JoystickButton4.whileTrue(new RotateToAngleCommand(swerveSubsystem,
        180, false));
        // Buttons.JoystickButton4
        //         .whileTrue(
        //                 new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.BARGE_MIDDLE,
        //                         candleSubsytem, driveCommands));

        Buttons.JoystickButton5
                .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.MIDDLE_ALGAE,
                        candleSubsytem, driveCommands)
                        // .andThen(new WaitCommand(0.2))
                        .andThen(driveCommands.driveToDistanceFieldOriented(0.25, -3))); //234
                        // .andThen(elevatorCommands.moveToIntakeCommand()));
        Buttons.JoystickButton5.onFalse(scoringCommands.stopScoringCoral());

        Buttons.JoystickButton6
                .whileTrue(
                        new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.LEFT_POLE,
                                candleSubsytem, driveCommands)
                                .andThen(scoringCommands.scoreCoralAuton()));
                                // .andThen(driveCommands.driveToDistanceFieldOriented(0.25, -1.5))
                                // .andThen(elevatorCommands.moveToIntakeCommand()));
        Buttons.JoystickButton6.onFalse(scoringCommands.stopScoringCoral());

        Buttons.JoystickButton7
                .whileTrue(
                        new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.RIGHT_POLE,
                                candleSubsytem, driveCommands)
                                .andThen(scoringCommands.scoreCoralAuton()));
                                // .andThen(driveCommands.driveToDistanceFieldOriented(0.25, -1.5))
                                // .andThen(elevatorCommands.moveToIntakeCommand()));
        Buttons.JoystickButton7.onFalse(scoringCommands.stopScoringCoral());

        Buttons.JoystickButton8.onTrue(hangCommands.pullInHang());
        Buttons.JoystickButton8.onTrue(elevatorCommands.moveToHangCommand());
        Buttons.JoystickButton8.onTrue(new InstantCommand(() -> hangSubsystem.releaseServo()));
        Buttons.JoystickButton8.onFalse(new InstantCommand(() -> hangSubsystem.setMotorSpeed(0.0)));

        // Buttons.JoystickButton9.onTrue(scoringCommands.scoreAlgaeDynamic());
        // Buttons.JoystickButton9.onFalse(scoringCommands.stopScoringAlgae());

        Buttons.JoystickButton9.onTrue(elevatorCommands.moveToL1Command());
        // Buttons.JoystickButton9.onFalse(scoringCommands.stopScoringAlgae());

        // Buttons.JoystickButton10
        //         .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.REEF_ANGLE,
        //                 candleSubsytem));

        Buttons.JoystickButton11.onTrue((Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance)));

        Buttons.JoystickButton12
                .onTrue(elevatorCommands.moveToAlgaeStateCommand(ElevatorState.LowAlgae, WristState.LowAlgae));
        Buttons.JoystickButton13
                .onTrue(elevatorCommands.moveToAlgaeStateCommand(ElevatorState.HighAlgae, WristState.HighAlgae));

        Buttons.JoystickButton14
                .onTrue(driveCommands.disableFrontCams());
                // .onTrue(driveCommands.disableFrontCams());
        Buttons.JoystickButton15
                .onTrue(driveCommands.enableFrontCams());
                // .onTrue(driveCommands.enableFrontCams());

                // Buttons.JoystickButton16
                // .onTrue(driveCommands.enableBackCam());

        // Buttons.JoystickButton14
        //         .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.BARGE_LEFT,
        //                 candleSubsytem));

        // Buttons.JoystickButton15
        //         .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.BARGE_MIDDLE,
        //                 candleSubsytem));

        // Buttons.JoystickButton16
        //         .whileTrue(new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.BARGE_RIGHT,
        //                 candleSubsytem));

        // Buttons.XboxXButton.onTrue(elevatorCommands.moveToL1Command());
        Buttons.XboxXButton.onTrue(elevatorCommands.moveToStateCommand(ElevatorState.BARGE, WristState.BARGE));

        Buttons.XboxAButton.onTrue(elevatorCommands.moveToL2Command());
        Buttons.XboxBButton.onTrue(elevatorCommands.moveToL3Command());
        Buttons.XboxYButton.onTrue(elevatorCommands.moveToL4Command());

        Buttons.XboxLeftStickButton.onTrue(intakeCommands.intakeCoral());

        // Buttons.XboxBackButton.onTrue(scoringCommands.scoreAlgaeDynamic());
        // Buttons.XboxBackButton.onFalse(scoringCommands.stopScoringAlgae());

        Buttons.XboxDPadNE.onTrue(intakeCommands.intakeAlgae());
        Buttons.XboxDPadNW.onTrue(intakeCommands.intakeAlgae());
        Buttons.XboxDPadN.onTrue(intakeCommands.intakeAlgae());

        Buttons.XboxDPadS
                .onTrue(elevatorCommands.moveToGroundAlgaeCommand());
        Buttons.XboxDPadSW
                .onTrue(elevatorCommands.moveToGroundAlgaeCommand());
        Buttons.XboxDPadSE
                .onTrue(elevatorCommands.moveToGroundAlgaeCommand());

        Buttons.XboxRightStickButton.onTrue(intakeCommands.stopIntakeCoral());
        Buttons.XboxRightStickButton.onTrue(intakeCommands.stopIntakeAlgae());

        Buttons.XboxLeftBumper.onTrue(hangCommands.pushOutHang());
        Buttons.XboxLeftBumper.onFalse(new InstantCommand(() -> hangSubsystem.setMotorSpeed(0.0)));
        // Buttons.XboxLeftBumper.onTrue(intakeCommands.reverseIntakeAlgae());

        Buttons.XboxStartButton.onTrue(elevatorCommands.moveToIntakeCommand());

        Buttons.XboxRightBumper
                .onTrue(elevatorCommands.moveToProcessorCommand());

        Buttons.XboxLeftTriggerButton
                .onTrue(elevatorCommands.moveToAlgaeStateCommand(ElevatorState.LowAlgae, WristState.LowAlgae));
        Buttons.XboxRightTriggerButton
                .onTrue(elevatorCommands.moveToAlgaeStateCommand(ElevatorState.HighAlgae, WristState.HighAlgae));

        Buttons.rightStickUp.whileTrue(elevatorCommands.moveElevatorUp());
        Buttons.rightStickDown.whileTrue(elevatorCommands.moveElevatorDown());

        Buttons.leftStickUp.whileTrue(elevatorCommands.moveWristUp());
        Buttons.leftStickDown.whileTrue(elevatorCommands.moveWristDown());

        // Buttons.XboxStartButton.whileTrue(driveCommands.driveUntilCANrangeCommand(AutoConstants.HumanPlayerCANrangeDist, 1));

        // Dash.add("HPS CANrange Dist", ()-> RobotMap.HPSrange.getDistanceInInches());


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

        // SmartDashboard.putBoolean("Flag", ()-> PhotonVision.);

        SmartDashboard.putNumber("IMU", swerveSubsystem.getGyro().getRawRotation3d().getZ());

        SmartDashboard.putData("Elevator L1",
                elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L1));
        SmartDashboard.putData("Elevator L2",
                elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L2));
        SmartDashboard.putData("Elevator L3",
                elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L3));
        SmartDashboard.putData("Elevator L4",
                elevatorCommands.moveElevatorToStateCommand(ElevatorSubsystem.ElevatorState.L4));
        SmartDashboard.putData("Elevator Up", elevatorCommands.moveElevatorUp());
        SmartDashboard.putData("Elevator Down", elevatorCommands.moveElevatorDown());

        SmartDashboard.putData("Intake Algae", intakeCommands.intakeAlgae());
        SmartDashboard.putData("Stop Algae", intakeCommands.stopIntakeAlgae());
        SmartDashboard.putData("Reverse Algae", intakeCommands.reverseIntakeAlgae());
    }
    // }

  /**
   * Register named commands for use in PathPlanner
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Score", scoringCommands.scoreCoralAuton());
    NamedCommands.registerCommand("L4Position", elevatorCommands.moveToL4Command());
    NamedCommands.registerCommand("IntakePosition", new InstantCommand(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
    .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
    .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.INTAKE), wristSubsystem))
    .andThen(Commands.waitSeconds(0.75))
    .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE)))
    .andThen(intakeCommands.intakeCoral()));
    NamedCommands.registerCommand("IntakeState", new InstantCommand(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
    .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
    .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.INTAKE), wristSubsystem))
    .andThen(Commands.waitSeconds(0.75))
    .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE))));
    NamedCommands.registerCommand("IntakeCoral", intakeCommands.intakeCoral());
    NamedCommands.registerCommand("WaitForIntake", new RunCommand(()-> System.out.println("Waiting")).until(()-> intakesubsystem.isDetectingCoral()).andThen(Commands.waitSeconds(1)));
    NamedCommands.registerCommand("DriveToReefRight",
    new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.RIGHT_POLE, candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToReefLeft",
        new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.LEFT_POLE, candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToHumanPlayer",
        new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.HUMAN_PLAYER_LEFT,
            candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToH1",
            new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.H1,
                candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToH4",
            new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.H4,
                candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToS2",
            new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.S2,
                candleSubsytem, driveCommands));
    NamedCommands.registerCommand("DriveToAlgae",
           new DriveToLocationAdvanced(swerveSubsystem, () -> aprilTagId, AlignLocation.MIDDLE_ALGAE, candleSubsytem, driveCommands)
           .andThen(driveCommands.driveToDistanceFieldOriented(0.25, -3)));
    NamedCommands.registerCommand("AlgaeStateLow",
    elevatorCommands.moveToAlgaeStateCommand(ElevatorState.LowAlgae, WristState.LowAlgae));
    NamedCommands.registerCommand("AlgaeStateHigh",
    elevatorCommands.moveToAlgaeStateCommand(ElevatorState.HighAlgae, WristState.HighAlgae));
    NamedCommands.registerCommand("BargeState", elevatorCommands.moveToStateCommand(ElevatorState.BARGE, WristState.BARGE));
    NamedCommands.registerCommand("BargeScore", new ParallelDeadlineGroup(new WaitCommand(0.2), scoringCommands.scoreAlgae()));
  }

    /**
     * Configure the dashboard with useful data for driving and debugging
     */
    private void setupDashboard() {
        autoChooser = AutoBuilder.buildAutoChooser();

        // Adds various data to the dashboard that is useful for driving and debugging
        SmartDashboard.putData("Auton Mode", autoChooser);
        Dash.add("CANrange Dist", () -> RobotMap.coralCANrange.getDistanceInInches());
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

        // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        // .withControllerHeadingAxis(Buttons.sidewaysSupplier, Buttons.forwardSupplier)
        // .headingWhile(true);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        // Command driveFieldOrientedDirectAngle =
        // driveCommands.driveFieldOriented(driveDirectAngle);

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
        // rotation supplier's value multiplied by π (Math.PI), and then scaled by 2π (2
        // * Math.PI).
        //
        // The sine function is used to calculate the x-axis component of the heading,
        // while the cosine function is used to calculate the y-axis component of the
        // heading.
        //
        // The headingWhile method is called with a true value to maintain the heading.
        //
        // @return A new SwerveInputStream with the specified controller heading axis
        // and heading behavior.
        // SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
        // .withControllerHeadingAxis(() -> Math.sin(
        // Buttons.rotateSupplier.getAsDouble() * Math.PI)
        // * (Math.PI * 2),
        // () -> Math.cos(
        // Buttons.rotateSupplier.getAsDouble() * Math.PI)
        // *
        // (Math.PI * 2))
        // .headingWhile(true);

        // Command driveFieldOrientedDirectAngleSim =
        // driveCommands.driveFieldOriented(driveDirectAngleSim);
        Command driveFieldOrientedAngularVelocitySim = driveCommands.driveFieldOriented(driveAngularVelocitySim);

        swerveSubsystem.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAngularVelocitySim);

        if (DriverStation.isTest()) {
            swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
        }

        // intakesubsystem.setDefaultCommand(
        //         new RunCommand(
        //                 () -> intakesubsystem.intakeCoral(Buttons.applyCurve(Buttons.XboxController.getLeftY(), Buttons.forwardCurve) * 0.25), 
        //         intakesubsystem));
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
    }
}
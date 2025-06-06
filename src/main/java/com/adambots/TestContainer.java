package com.adambots;

import com.adambots.commands.elevatorCommands.ElevatorCommands;
import com.adambots.commands.intakeCommands.IntakeCommands;
import com.adambots.commands.scoringCommands.ScoringCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.WristSubsystem;

import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is an alternative to RobotContainer for testing purposes. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 * 
 * When we use a test board, we can't use the RobotContainer as it will fail due to the lack of a physical robot.
 * Hence, enable a TEST_CONTAINER flag in the Robot class to use this class instead.
 */
public class TestContainer {

  // The robot's subsystems are defined here...
  // private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs);
  // IntakeSubsystem intakesubsystem = new IntakeSubsystem(RobotMap.minionMotor,
  // RobotMap.algaeGripper, RobotMap.algaeRunner, RobotMap.coralCANrange);  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotMap.elevatorMotor);
  IntakeSubsystem intakesubsystem = new IntakeSubsystem(RobotMap.minionMotor,
    RobotMap.algaeLeftMotor, RobotMap.algaeRightMotor, RobotMap.coralCANrange, RobotMap.algaeCANrange);

  //   WristSubsystem wristSubsystem = new WristSubsystem(RobotMap.wristMotor, RobotMap.wristEncoder);
  private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs);


  // Add commands here
  // private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);
  private final IntakeCommands intakeCommands = new IntakeCommands(intakesubsystem, candleSubsytem);
  // private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem, wristSubsystem, intakeCommands);
  // private final ScoringCommands scoringCommands = new ScoringCommands(intakesubsystem, ElevatorSubsystem, candleSubsytem);

  // Creates a SmartDashboard element to allow drivers to select differnt autons
  // private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public TestContainer() {
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
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   * If you are not using a Joystick or XboxController, you can add commands to the shuffleboard
   * Use: SmartDashboard.putData("Command Name", new Command());
   */
  private void configureButtonBindings() {
    // JOYSTICK BINDINGS SHOULD BE IN NUMERICAL ORDER TO PREVENT DOUBLE BINDINGS

    if (Robot.isSimulation()) {
    
    }
    SmartDashboard.putData("Run Intake", intakeCommands.intakeCoral());
    SmartDashboard.putData("Stop Intake", intakeCommands.stopIntakeCoral());
    // SmartDashboard.putData("Shoot Coral", scoringCommands.scoreCoral());

    if (DriverStation.isTest()) {
      
    } else {
        // Buttons.XboxLeftStickButton.onTrue(intakeCommands.intakeCoral());
        // Buttons.XboxLeftStickButton.onTrue(Commands.runOnce(()->intakesubsystem.intakeCoral()));
        // SmartDashboard.putData(Commands.runOnce(()->intakesubsystem.intakeCoral()));

        

    // SmartDashboard.putData("Intake Coral", intakeCommands.intakeCoral());
    // SmartDashboard.putData("Stop Intake Coral", intakeCommands.stopIntakeCoral());
    // SmartDashboard.putData("Slow Intake Coral", intakeCommands.slowIntakeCoral());
    // SmartDashboard.putData("Reverse Intake Coral", intakeCommands.reverseIntakeCoral());

    Dash.add("Distance", () -> RobotMap.coralCANrange.getDistanceInCentimeters());

    // SmartDashboard.putData("Intake State", elevatorCommands.moveToIntakeCommand());
    // SmartDashboard.putData("L1 State", elevatorCommands.moveToL1Command());
    // SmartDashboard.putData("L2 State", elevatorCommands.moveToL2Command());
    // SmartDashboard.putData("L3 State", elevatorCommands.moveToL3Command());
    // SmartDashboard.putData("L4 State", elevatorCommands.moveToL4Command());

    // SmartDashboard.putData("Wrist Stowed", elevatorCommands.moveWristToIntakeCommand());
    // SmartDashboard.putData("Wrist Intake", elevatorCommands.moveWristToStowedCommand());
    // SmartDashboard.putData("Wrist L1", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L1));
    // SmartDashboard.putData("Wrist L2", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L2));
    // SmartDashboard.putData("Wrist L3", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L3));
    // SmartDashboard.putData("Wrist L4", elevatorCommands.moveWristToStateCommand(WristSubsystem.WristState.L4));

    SmartDashboard.putData("Intake Algae", intakeCommands.intakeAlgae());
    SmartDashboard.putData("Stop Algae", intakeCommands.stopIntakeAlgae());
    SmartDashboard.putData("Reverse Algae", intakeCommands.reverseIntakeAlgae());

    // SmartDashboard.putData("Score", scoringCommands.scoreCoral());

    // SmartDashboard.putData("Servo test", Commands.runOnce(()-> RobotMap.bottomCoralActuator.set(1)));
    // Dash.add("isDetecting", () -> intakesubsystem.isDetectingCoral());

  //   SmartDashboard.putData(Commands.run( ()-> {
  //     SmartDashboard.putBoolean("LimitSwitch1", RobotMap.firstIntakeLimit.isDetecting());
  //     SmartDashboard.putBoolean("LimitSwitch2", RobotMap.secondIntakeLimit.isDetecting());

  //   }));
    }
  }

  /**
   * Register named commands for use in PathPlanner
   */
  private void registerNamedCommands() {
    
  }

  /**
   * Configure the dashboard with useful data for driving and debugging
   */
  private void setupDashboard() {
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Adds various data to the dashboard that is useful for driving and debugging
    // SmartDashboard.putData("Auton Mode", autoChooser);

    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());
  }

  private void setupDefaultCommands() {
   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return Commands.none();
  }
}
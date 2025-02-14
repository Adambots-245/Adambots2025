package com.adambots;

import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.TestArmSubsystem;
import com.adambots.subsystems.TestArmSubsystem2;
import com.adambots.utils.Buttons;
import com.adambots.utils.Dash;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
  private final TestArmSubsystem2 testArmSubsystem = new TestArmSubsystem2();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // Add commands here
  // private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);

  // Creates a SmartDashboard element to allow drivers to select differnt autons
  // private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private SendableChooser<Command> autoChooser;

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

    // if (Robot.isSimulation()) 
    {
      // Buttons.XboxAButton.onTrue(testArmSubsystem.moveToLowCommand());
      // Buttons.XboxBButton.onTrue(testArmSubsystem.moveToMidCommand());
      // Buttons.XboxXButton.onTrue(testArmSubsystem.moveToHighCommand());
      Buttons.XboxAButton.onTrue(elevatorSubsystem.moveToIntakeCommand());
      Buttons.XboxBButton.onTrue(elevatorSubsystem.moveToL1Command());
      Buttons.XboxYButton.onTrue(elevatorSubsystem.moveToL2Command());
      Buttons.XboxXButton.onTrue(elevatorSubsystem.moveToL3Command());
      Buttons.XboxDPadN.onTrue(elevatorSubsystem.moveToL4Command());

    }

    if (DriverStation.isTest()) {
      
    } else {
      
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
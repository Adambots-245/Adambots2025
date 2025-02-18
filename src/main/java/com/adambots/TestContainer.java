package com.adambots;

import com.adambots.subsystems.TestSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.utils.Dash;
import com.adambots.utils.TestStates;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

  // Add commands here
  // private final DriveCommands driveCommands = new DriveCommands(swerveSubsystem);

  // Creates a SmartDashboard element to allow drivers to select differnt autons
  // private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private TestSubsystem testSubsystem = new TestSubsystem();

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

    if (Robot.isSimulation()) {
    
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
    // SmartDashboard.putData("servo", Commands.runOnce(() -> {
    //   RobotMap.servo.set(0.5);
    // }));
    // SmartDashboard.putData("servo Angle", Commands.runOnce(() -> {
    //   RobotMap.servo.setAngle(90);
    // }));
    //     SmartDashboard.putData("set 45", Commands.runOnce(() -> {
    //   RobotMap.servo.setAngle(45);
    // }));
    // SmartDashboard.putData("set 1", Commands.runOnce(() -> {
    //   RobotMap.servo.set(1);
    // }));
    // SmartDashboard.putData("set 0", Commands.runOnce(() -> {
    //   RobotMap.servo.set(0);
    // }));
    // SmartDashboard.putData("negativeSpeed", Commands.runOnce(() -> {
    //   RobotMap.servo.setSpeed(-1);;
    // }));
    // SmartDashboard.putData("fullSpeed", Commands.runOnce(() -> {
    //   RobotMap.servo.setSpeed(1);;
    // }));
    // SmartDashboard.putData("STOP", Commands.runOnce(() -> {
    //   RobotMap.servo.setSpeed(0);;
    // }));
    // Dash.add("getSpeed", () -> RobotMap.servo.getSpeed());

    // SmartDashboard.putNumber("analog pos", RobotMap.servoAnalog.getValue());
    // Dash.add("analog", () -> RobotMap.servoAnalog.getValue());

    // SmartDashboard.putData("set hub speed", Commands.runOnce(() -> RobotMap.hubServo.turnClockwise()));
    
    SmartDashboard.putData("Run intake", Commands.runOnce(
      () -> {
        RobotMap.hubServo.turnCounterclockwise();
        RobotMap.hubServo2.turnClockwise();
      }
    ));

    SmartDashboard.putData("Reverse intake", Commands.runOnce(
      () -> {
        RobotMap.hubServo.turnClockwise();
        RobotMap.hubServo2.turnCounterclockwise();
      }
    ));

    SmartDashboard.putData("Stop intake", Commands.runOnce(
      () -> {
        RobotMap.hubServo.stop();
        RobotMap.hubServo2.stop();
      }
    ));

    SmartDashboard.putData("Stop Algae", Commands.runOnce(
      () -> {
        // RobotMap.hubServo3.stop();
        
        RobotMap.hubServo4.stop();
      }, testSubsystem
    ));

    SmartDashboard.putData("Run Algae Intake", Commands.repeatingSequence(
      Commands.runOnce(
      () -> {
        System.out.println("Timer 1:" + Timer.getFPGATimestamp());
        RobotMap.hubServo3.turnCounterclockwise();
        // RobotMap.hubServo3.setPulseWidth(4500);
        RobotMap.hubServo4.turnClockwise();
      }, testSubsystem
    ).andThen(
      Commands.waitSeconds(0.5)
    ).andThen(Commands.runOnce(
      () -> {
        System.out.println("Timer 2:" + Timer.getFPGATimestamp());

        RobotMap.hubServo4.stop();
      }  
    )
    )
    ));

    SmartDashboard.putData("Reverse Algae Intake", Commands.runOnce(
      () -> {
        RobotMap.hubServo3.turnClockwise();
        RobotMap.hubServo4.turnCounterclockwise();
      }, testSubsystem
    ));

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
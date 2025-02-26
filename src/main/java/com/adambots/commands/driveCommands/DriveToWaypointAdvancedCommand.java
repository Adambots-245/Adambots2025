package com.adambots.commands.driveCommands;

import com.adambots.RobotContainer;
import com.adambots.Constants.DriveConstants;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.utils.Buttons;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;

public class DriveToWaypointAdvancedCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private PIDController xController = new PIDController(2.5, 0, 0);
  private PIDController yController = new PIDController(2.5, 0,0);
  // private PIDController thetaController = new PIDController(5, 0, 0.02);

  // private PIDController xController = new PIDController(1, 0, 0);
  // private PIDController yController = new PIDController(1, 0,0);
  private PIDController thetaController = new PIDController(5, 0, 0.02);

  private Pose2d waypoint;

  private int[] redTagIDs = {6, 7, 8, 9, 10, 11};
  private int[] blueTagIDs = {17, 18, 19, 20, 21, 22};
  private double offset = 0.5;

  private boolean isSeen = false;
  private int idSeen = -1;
  private boolean isOffsetRight;
  private int reefAngle;

  PathConstraints constraints;

  private Pose2d offsetWaypoint;

  private boolean isCalculated = false;

  private double xPos;
  private double yPos;

  private double drive_output_x;
  private double drive_output_y;
  private double drive_output_theta;

  public DriveToWaypointAdvancedCommand(SwerveSubsystem swerveSubsystem, boolean isOffsetRight) {
    // addRequirements(swerveSubsystem);

    this.swerveSubsystem = swerveSubsystem;
    this.isOffsetRight = isOffsetRight;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    constraints = new PathConstraints(
      4, 3.0,
      swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    waypoint = new Pose2d(new Translation2d(13.728, 2.884), new Rotation2d(Math.toRadians(120)));
    xPos = 0;
    yPos = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    xPos = swerveSubsystem.getPose().getX();
    yPos = swerveSubsystem.getPose().getY();

    idSeen = swerveSubsystem.getVision().hasID(redTagIDs);

    if (idSeen > -1) {
      isSeen = true;
    }

    if (isSeen == true && isCalculated == false){
      System.out.println(idSeen);
      switch(idSeen) {
        case 6:
          System.out.println("CASE 6");
          waypoint = new Pose2d(new Translation2d(13.728, 2.884), new Rotation2d(Math.toRadians(120)));
          reefAngle = 30;
          break;
        case 7:
          waypoint = new Pose2d(new Translation2d(14.281, 4.025), new Rotation2d(Math.toRadians(180)));
          break;
        case 8:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 9:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 10:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 11:
          waypoint = new Pose2d(new Translation2d(12.412, 2.913), new Rotation2d(Math.toRadians(60)));
          break;
        case 17:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 18:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 19:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 20:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 21:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
        case 22:
          waypoint = new Pose2d(new Translation2d(), new Rotation2d());
          break;
      }

      if (isOffsetRight) {
        offsetWaypoint = new Pose2d(new Translation2d(waypoint.getX() - offset*Math.cos(Math.toRadians(reefAngle)), waypoint.getY() - offset*Math.sin(Math.toRadians(reefAngle))), waypoint.getRotation());
      } else {
        offsetWaypoint = new Pose2d(new Translation2d(waypoint.getX() + offset*Math.cos(Math.toRadians(reefAngle)), waypoint.getY() + offset*Math.sin(Math.toRadians(reefAngle))), waypoint.getRotation());
      }

      isCalculated = true;
    } 

    AutoBuilder.pathfindToPose(
                waypoint,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        ).schedule();

    if (isCalculated){
      
      // drive_output_x = xController.calculate(xPos, waypoint.getX());
      // drive_output_y = yController.calculate(yPos, waypoint.getY());

      // drive_output_x = xController.calculate(0, waypoint.getX()-xPos);
      // drive_output_y = yController.calculate(0, waypoint.getY()-yPos);

      // drive_output_theta = thetaController.calculate(swerveSubsystem.getHeading().getRadians(), waypoint.getRotation().getRadians());

      // // System.out.println("YPos: " + yPos);
      // // System.out.println("xPos: " + xPos);

      // System.out.println("Get Heading Swerve " + swerveSubsystem.getHeading().getRadians());
      // System.out.println("Goal Heading " + waypoint.getRotation().getRadians());

      // // System.out.println("Waypoint YPos: " + waypoint.getY());

      // swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(drive_output_x, drive_output_y, drive_output_theta, swerveSubsystem.getHeading()));
      // swerveSubsystem.drive(new ChassisSpeeds(-drive_output_x,-drive_output_y,drive_output_theta));
      // swerveSubsystem.drive(new ChassisSpeeds(-drive_output_x,0,drive_output_theta));
      // swerveSubsystem.drive(new ChassisSpeeds(0,-drive_output_y,drive_output_theta));
    }

    
    // System.out.println("X Distance " + (Math.abs(waypoint.getX() - xPos)));
    // System.out.println("Y Distance " + (Math.abs(waypoint.getY() - yPos)));

    // System.out.println(swerveSubsystem.getVision().hasID(redTagIDs));
    


      // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      //   ()-> Buttons.forwardSupplier.getAsDouble(),
      //   ()-> Buttons.sidewaysSupplier.getAsDouble())
      //   .withControllerRotationAxis(Buttons.rotateSupplier)
      //   .deadband(DriveConstants.kDeadZone)
      //   .scaleTranslation(0.8)
      //   .allianceRelativeControl(true);
      // driveCommands.driveFieldOriented(driveAngularVelocity);
      // swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.getDriveAngularVelocity().get(), swerveSubsystem.getHeading()));
    // }
    

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

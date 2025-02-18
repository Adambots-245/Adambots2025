package com.adambots.commands.driveCommands;

import com.adambots.RobotContainer;
import com.adambots.Constants.DriveConstants;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;

public class DriveToWaypointCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private PIDController xController = new PIDController(2.5, 0, 0);
  private PIDController yController = new PIDController(2.5, 0,0);
  private PIDController thetaController = new PIDController(5, 0, 0.02);
  private Pose2d waypoint;

  private double xPos;
  private double yPos;

  private double drive_output_x;
  private double drive_output_y;
  private double drive_output_theta;

  private int framesSeen = 0;

  private DriveCommands driveCommands;

  // private RobotContainer robotContainer = new RobotContainer();

  public DriveToWaypointCommand(SwerveSubsystem swerveSubsystem, Pose2d waypoint, DriveCommands driveCommands) {
    addRequirements(swerveSubsystem);

    this.swerveSubsystem = swerveSubsystem;
    this.waypoint = waypoint;
    this.driveCommands = driveCommands;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    framesSeen = 0;

    xPos = 0;
    yPos = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    xPos = swerveSubsystem.getPose().getX();
    yPos = swerveSubsystem.getPose().getY();

    drive_output_x = xController.calculate(xPos, waypoint.getX());
    drive_output_y = yController.calculate(yPos, waypoint.getY());
    drive_output_theta = thetaController.calculate(swerveSubsystem.getHeading().getRadians(), waypoint.getRotation().getRadians());
    

    if(swerveSubsystem.getVision().hasTarget()){
    //   framesGone = 0;
    // } else {
      framesSeen++;
    }

    System.out.println(framesSeen);
    
    if(framesSeen > 10){
      // swerveSubsystem.drive(new ChassisSpeeds(-drive_output_x,-drive_output_y,drive_output_theta));
    }else{
      // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      //   ()-> Buttons.forwardSupplier.getAsDouble(),
      //   ()-> Buttons.sidewaysSupplier.getAsDouble())
      //   .withControllerRotationAxis(Buttons.rotateSupplier)
      //   .deadband(DriveConstants.kDeadZone)
      //   .scaleTranslation(0.8)
      //   .allianceRelativeControl(true);
      
      // driveCommands.driveFieldOriented(driveAngularVelocity);
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.getDriveAngularVelocity().get(), swerveSubsystem.getHeading()));


      //allow human control

    }
    
    // System.out.println("X err: " + xController.getError() + " | Y err: " + yController.getError() + " | Theta err: " + thetaController.getError() + " | Inc " + finishedInc);
      
    // if (xPos != 0 && yPos != 0) {
    //   double xDrive = xController.calculate(xPos);
    //   double yDrive = yController.calculate(yPos);
      
    //   if ((Math.abs(waypoint.getX()-xPos) > 1.25 || Math.abs(waypoint.getY()-yPos) > 0.7) && waypoint != AutoConstants.S2_POSE2D){
    //     if (Utils.isOnRedAlliance()) { //If the robot is further than 1.25 meters in x, rotate to face the apriltags instead of the waypoint to maintain them in FOV
    //       thetaController.setSetpoint(Math.atan2(VisionConstants.aprilTagPos.getY()-yPos, VisionConstants.aprilTagPos.getX()+VisionConstants.kFieldWidth-xPos) + Math.PI);
    //     } else {
    //       thetaController.setSetpoint(Math.atan2(VisionConstants.aprilTagPos.getY()-yPos, VisionConstants.aprilTagPos.getX()-xPos) + Math.PI);
    //     }
    //   } else {
    //     if (Utils.isOnRedAlliance()) {
    //       thetaController.setSetpoint(waypoint.getRotation().getRadians() + Math.PI);
    //     } else {
    //       thetaController.setSetpoint(waypoint.getRotation().getRadians());
    //     }
    //   }
    //   double thetaDrive = thetaController.calculate(gyro.getContinuousYawRad());

    //   if (getDist(waypoint, drivetrainSubsystem.getPose()) > 1.6) {
    //     xDrive = MathUtil.clamp(xDrive, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);
    //     yDrive = MathUtil.clamp(yDrive, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);
    //   } else {
    //     xDrive = MathUtil.clamp(xDrive, -AutoConstants.kMinWaypointTranslateSpeed, AutoConstants.kMinWaypointTranslateSpeed);
    //     yDrive = MathUtil.clamp(yDrive, -AutoConstants.kMinWaypointTranslateSpeed, AutoConstants.kMinWaypointTranslateSpeed);
    //   }
      
    //   if (Utils.isOnRedAlliance()) {
    //     drivetrainSubsystem.drive(-xDrive, -yDrive, thetaDrive, true);
    //   } else {
    //     drivetrainSubsystem.drive(xDrive, yDrive, thetaDrive, true);
    //   }

    //   if (waypoint.equals(AutoConstants.S2_POSE2D)) {
    //     if (Math.abs(xController.getError()) < 0.35 && Math.abs(yController.getError()) < 0.15 && Math.abs(thetaController.getError()) < Math.toRadians(3)) {
    //       finishedInc++;
    //     } else if (finishedInc > 0) {
    //       finishedInc--;
    //     }
    //   } else {
    //     if (Math.abs(xController.getError()) < 0.15 && Math.abs(yController.getError()) < 0.15 && Math.abs(thetaController.getError()) < Math.toRadians(3)) {
    //       finishedInc++;
    //     } else if (finishedInc > 0) {
    //       finishedInc--;
    //     }
      // }
    }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  //   if (abortInc > abortThreshold) {
  //     System.out.println(this.getName() + " | ABORTED - UNRELIABLE APRILTAG DETECTION");
  //     return true;
  //   }
  //   return finishedInc > 10;
  // }

  // public double getDist (Pose2d pos1, Pose2d pos2) {
  //   double x = pos1.getX() - pos2.getX();
  //   double y = pos1.getY() - pos2.getY();
        
  //   return Math.hypot(x, y);
  // }

  // public double getContinuousAngleError (double setpoint, double measurement) {
  //   return MathUtil.inputModulus(setpoint - measurement, -Math.PI, Math.PI);
  }
}

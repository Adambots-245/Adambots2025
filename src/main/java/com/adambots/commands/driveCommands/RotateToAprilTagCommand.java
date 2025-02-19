package com.adambots.commands.driveCommands;

import com.adambots.Constants.DriveConstants;
import com.adambots.Robot;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.utils.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAprilTagCommand extends Command {

  private SwerveSubsystem swerveSubsystem;
  private PIDController angleTurningPIDController = new PIDController(5, 0, 0.02);
  private double drive_output;

  public RotateToAprilTagCommand(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);

    angleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves left or right depending on the angle
    drive_output = angleTurningPIDController.calculate(swerveSubsystem.getHeading().getRadians(), swerveSubsystem.getAprilTagYaw(7).getRadians());
    swerveSubsystem.drive(new ChassisSpeeds(0,0,drive_output));
    // System.out.println("Working");
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

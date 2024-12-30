package com.adambots.commands.driveCommands;
import com.adambots.Constants.DriveConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.wpilibj2.command.Command;

public class SpinCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;

  public SpinCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Moves left or right depending on the angle
    driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , DriveConstants.kTeleopRotationalSpeed * 0.6, true);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If the robot is aligned for some time, or the robot is not detecting a piece the command ends
    return false;
  }
}

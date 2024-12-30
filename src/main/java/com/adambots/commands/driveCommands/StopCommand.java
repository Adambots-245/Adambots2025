// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.driveCommands;

import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class StopCommand extends Command {
  /** Creates a new StopCommand. */
  private DrivetrainSubsystem drivetrainSubsystem;

  public StopCommand(DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(drivetrainSubsystem);

    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

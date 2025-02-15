// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.driveCommands;

import com.adambots.subsystems.ElevatorTestSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTestCommands extends Command {
 ElevatorTestSubsystem elevatorTestSubsystem;
  /** Creates a new MoveToLowerCommand. */
  public ElevatorTestCommands(ElevatorTestSubsystem elevatorTestSubsystem) {
    this.elevatorTestSubsystem = elevatorTestSubsystem;
    addRequirements(elevatorTestSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command moveToLower() {
    return Commands.runOnce(() -> {
      elevatorTestSubsystem.moveToLowerState();
    }, elevatorTestSubsystem);
  }

  public Command moveToUpper() {
    return Commands.runOnce(() -> {
      elevatorTestSubsystem.moveToUpperState();
    }, elevatorTestSubsystem);
  }
}

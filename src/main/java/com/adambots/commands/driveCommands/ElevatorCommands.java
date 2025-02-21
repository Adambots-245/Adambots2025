// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.driveCommands;

import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.ElevatorSubsystem.ElevatorState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {
  /** Creates a new ElevatorCommands. */
  ElevatorSubsystem elevatorSubsystem;

  public ElevatorCommands(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.elevatorSubsystem = elevatorSubsystem;
  }

  public Command moveToIntakeCommand() {
        return Commands.runOnce(() -> elevatorSubsystem.moveToState(ElevatorState.INTAKE), elevatorSubsystem);
    }

    public Command moveToL1Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveToState(ElevatorState.L1), elevatorSubsystem);
    }

    public Command moveToL2Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveToState(ElevatorState.L2), elevatorSubsystem);
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveToState(ElevatorState.L3), elevatorSubsystem);
    }

    public Command moveToL4Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveToState(ElevatorState.L4), elevatorSubsystem);
    }
}

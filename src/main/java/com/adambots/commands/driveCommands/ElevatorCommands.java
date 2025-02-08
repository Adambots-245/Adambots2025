// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.driveCommands;

import com.adambots.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {
  ElevatorSubsystem elevatorSubsystem;

  /** Creates a new ElevatorCommands. */
  public ElevatorCommands(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command goToIntake() {
    return Commands.runOnce(() -> {
      elevatorSubsystem.moveToIntake();
    }, elevatorSubsystem);
  }

  public Command goToL1() {
    return Commands.runOnce(() -> {
      elevatorSubsystem.moveToL1();
    }, elevatorSubsystem);
  }

  public Command goToL2() {
    return Commands.runOnce(() -> {
      elevatorSubsystem.moveToL2();
    }, elevatorSubsystem);
  }

  public Command goToL3() {
    return Commands.runOnce(() -> {
      elevatorSubsystem.moveToL3();
    }, elevatorSubsystem);
  }

  public Command goToL4() {
    return Commands.runOnce(() -> {
      elevatorSubsystem.moveToL4();
    }, elevatorSubsystem);
  }
}

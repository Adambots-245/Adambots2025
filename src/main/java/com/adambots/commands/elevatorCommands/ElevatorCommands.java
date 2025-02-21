// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.elevatorCommands;

import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.ElevatorSubsystem.ElevatorState;
import com.adambots.subsystems.ElevatorSubsystem.WristState;

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
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.INTAKE), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.INTAKE),
                        elevatorSubsystem));
    }

    public Command moveToL1Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L1), elevatorSubsystem)
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L1),
                        elevatorSubsystem));
    }
    
    public Command moveToL2Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L2), elevatorSubsystem)
        .alongWith(Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L2),
                        elevatorSubsystem));
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L3), elevatorSubsystem)
        .alongWith(Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L3),
                        elevatorSubsystem));
    }

    public Command moveToL4Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L4), elevatorSubsystem)
        .alongWith(Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L4),
                        elevatorSubsystem));
    }

    public Command moveWristToStateCommand(WristState state) {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(state), elevatorSubsystem);
    }

    public Command moveElevatorToStateCommand(ElevatorState state) {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(state), elevatorSubsystem);
    }

    public Command moveWristToStowedCommand() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem);
    }

    public Command moveWristToIntakeCommand() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.INTAKE), elevatorSubsystem);
    }

    public Command moveWristToL1Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L1), elevatorSubsystem);
    }

    public Command moveWristToL2Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L2), elevatorSubsystem);
    }

    public Command moveWristToL3Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L3), elevatorSubsystem);
    }

    public Command moveWristToL4Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveWristToState(WristState.L4), elevatorSubsystem);
    }
}

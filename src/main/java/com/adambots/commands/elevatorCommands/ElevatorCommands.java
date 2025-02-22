// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.elevatorCommands;

import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.ElevatorSubsystem.ElevatorState;
import com.adambots.subsystems.WristSubsystem;
import com.adambots.subsystems.WristSubsystem.WristState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {
    /** Creates a new ElevatorCommands. */
    ElevatorSubsystem elevatorSubsystem;
    WristSubsystem wristSubsystem;

    public ElevatorCommands(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    public Command moveToIntakeCommand() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.INTAKE), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE), wristSubsystem));
    }

    public Command moveToL1Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L1), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L1),
                        wristSubsystem));
    }

    public Command moveToL2Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L2), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L2),
                        wristSubsystem));
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L3), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L3),
                        wristSubsystem));
    }

    public Command moveToL4Command() {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L4), elevatorSubsystem)
                .alongWith(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L4),
                        wristSubsystem));
    }

    public Command moveWristToStateCommand(WristState state) {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(state), wristSubsystem);
    }

    public Command moveElevatorToStateCommand(ElevatorState state) {
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(state), elevatorSubsystem);
    }

    public Command moveWristToStowedCommand() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), wristSubsystem);
    }

    public Command moveWristToIntakeCommand() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE), wristSubsystem);
    }

    public Command moveWristToL1Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L1), wristSubsystem);
    }

    public Command moveWristToL2Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L2), wristSubsystem);
    }

    public Command moveWristToL3Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L3), wristSubsystem);
    }

    public Command moveWristToL4Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L4), wristSubsystem);
    }

    public Command moveElevatorUp(){
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorUp(), elevatorSubsystem);
    }

    public Command moveElevatorDown(){
        return Commands.runOnce(() -> elevatorSubsystem.moveElevatorDown(), elevatorSubsystem);
    }
}
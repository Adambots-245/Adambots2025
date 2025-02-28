// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.elevatorCommands;

import com.adambots.Constants.ElevatorConstants;
import com.adambots.commands.intakeCommands.IntakeCommands;
import com.adambots.subsystems.ElevatorSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
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
    IntakeCommands intakeCommands;

    public ElevatorCommands(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            IntakeCommands intakeCommands) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.intakeCommands = intakeCommands;
    }

    public Command moveToIntakeCommand() {
        return Commands.either(
            Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE))
                .andThen(intakeCommands.intakeCoral())
            , 
            Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.INTAKE), wristSubsystem))
                .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.INTAKE)))
                .andThen(intakeCommands.intakeCoral()), 
            ()-> elevatorSubsystem.getCurrentElevatorState().equals(ElevatorState.INTAKE));
    }

    public Command moveToL1Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L1), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L1))));
    }

    public Command moveToL2Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L2), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L2))));
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L3), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L3))));
    }

    public Command moveToL4Command() {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateFirstChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(ElevatorState.L4), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.L4))));
    }

    public Command moveToAlgaeStateCommand(ElevatorState elevatorState, WristState wristState) {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(elevatorState), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(wristState))))
                .andThen(intakeCommands.intakeAlgae());
    }

    public Command moveToStateCommand(ElevatorState elevatorState, WristState wristState) {
        return Commands.runOnce(() -> wristSubsystem.moveWristToState(WristState.STOWED), elevatorSubsystem)
                .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                .andThen(Commands.runOnce(() -> elevatorSubsystem.moveElevatorToState(elevatorState), wristSubsystem)
                        .andThen(Commands.waitSeconds(ElevatorConstants.stateChangeDelay))
                        .andThen(Commands.runOnce(() -> wristSubsystem.moveWristToState(wristState))));
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
        return Commands.runEnd(() -> wristSubsystem.moveWristToState(WristState.INTAKE),
                () -> wristSubsystem.holdWristPosition(), wristSubsystem);
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

    public Command moveElevatorUp() {
        // return Commands.runEnd(() -> elevatorSubsystem.moveElevatorUp(), ()->
        // elevatorSubsystem.stopElevatorSpeed(), elevatorSubsystem);
        return Commands.runEnd(() -> elevatorSubsystem.moveElevatorUp(), () -> elevatorSubsystem.holdElevatorPosition(),
                elevatorSubsystem);
    }

    public Command moveElevatorDown() {
        // return Commands.runEnd(() -> elevatorSubsystem.moveElevatorDown(), ()->
        // elevatorSubsystem.stopElevatorSpeed(), elevatorSubsystem);
        return Commands.runEnd(() -> elevatorSubsystem.moveElevatorDown(),
                () -> elevatorSubsystem.holdElevatorPosition(), elevatorSubsystem);
    }

    public Command moveWristUp() {
        return Commands.runEnd(() -> wristSubsystem.moveWristUp(), () -> wristSubsystem.holdWristPosition(),
                wristSubsystem);
    }

    public Command moveWristDown() {
        return Commands.runEnd(() -> wristSubsystem.moveWristDown(), () -> wristSubsystem.holdWristPosition(),
                wristSubsystem);
    }
}
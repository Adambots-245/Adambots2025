package com.adambots.commands.intakeCommands;

import com.adambots.Constants.IntakeConstants;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command intake() {
        return Commands.runOnce(() -> {
                System.out.println("Intake Command Running");
                intakeSubsystem.intake();
            }, intakeSubsystem)
            .andThen(Commands.waitUntil(intakeSubsystem::isDetecting))
            .andThen(Commands.waitSeconds(IntakeConstants.kTimerThreshold))
            .andThen(Commands.runOnce(intakeSubsystem::stopIntake, intakeSubsystem));
    }

    public Command reverseIntake() {
        return Commands.runOnce(
            intakeSubsystem::reverseIntake,
            intakeSubsystem
        );
    }

    public Command slowIntake() {
        return Commands.runOnce(
            intakeSubsystem::slowIntake,
            intakeSubsystem
        );
    }

    public Command stopIntake() {
        return Commands.runOnce(
            intakeSubsystem::stopIntake,
            intakeSubsystem
        );
    }
}
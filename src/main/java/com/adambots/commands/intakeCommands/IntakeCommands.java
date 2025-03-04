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

    public Command intakeCoral() {
        return Commands.runOnce(() -> {
                System.out.println("Intake Command Running");
                intakeSubsystem.intakeCoral();
            }, intakeSubsystem)
            .andThen(Commands.waitUntil(intakeSubsystem::isDetectingCoral)).andThen(() -> System.out.println("DETECTED"))
            .andThen(Commands.waitSeconds(IntakeConstants.kTimerThreshold)).andThen(() -> System.out.println("STOPPED"))
            .andThen(Commands.runOnce(intakeSubsystem::stopCoralIntake));
    }

    public Command reverseIntakeCoral() {
        return Commands.runOnce(
            intakeSubsystem::reverseCoralIntake,
            intakeSubsystem
        );
    }

    public Command slowIntakeCoral() {
        return Commands.runOnce(
            intakeSubsystem::slowCoralIntake,
            intakeSubsystem
        );
    }

    public Command stopIntakeCoral() {
        return Commands.runOnce(
            intakeSubsystem::stopCoralIntake, intakeSubsystem
        );
    }

    public Command intakeAlgae() {
        return Commands.runOnce(
            intakeSubsystem::intakeAlgae, intakeSubsystem
        );
    }
    
    public Command stopIntakeAlgae() {
        return Commands.runOnce(
            intakeSubsystem::stopAlgaeIntake, intakeSubsystem
        );
    }

    public Command reverseIntakeAlgae() {
        return Commands.runOnce(
            intakeSubsystem::reverseAlgaeIntake
        );
    }
}
package com.adambots.commands.intakeCommands;

import com.adambots.Constants.IntakeConstants;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
    private final IntakeSubsystem intakeSubsystem;
    private int counter;

    public IntakeCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command intake() {
        return new FunctionalCommand(
            // initialize
            () -> {
                System.out.println("Intake Command Running");
                counter = 0;
                intakeSubsystem.intake();
            },
            // execute
            () -> {
                if (intakeSubsystem.isDetecting())
                    counter++;
            },
            // end
            interrupted -> intakeSubsystem.stopIntake(),
            // isFinished
            () -> counter > IntakeConstants.kTimerThreshold,
            // requirements
            intakeSubsystem
        );
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
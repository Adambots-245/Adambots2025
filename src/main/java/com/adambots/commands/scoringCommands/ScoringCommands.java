// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.scoringCommands;

import com.adambots.Constants.IntakeConstants;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Commands for scoring mechanisms
 */
public class ScoringCommands {

    private final IntakeSubsystem intakeSubsystem;

    public ScoringCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command scoreCoral() {
        // No coral - don't do anything
        // if (!intakeSubsystem.isDetectingCoral()) {
        //     System.out.println("Score stopped");
        //     return Commands.none();
        // }

        // run the intake, wait until the CANRange does not see a Coral, wait a bit,
        // then stop the intake
        return Commands.runOnce(() -> {
            System.out.println("Score Command Running");
            intakeSubsystem.intakeCoral();
        }, intakeSubsystem).andThen(Commands.waitSeconds(2))
                // .andThen(Commands.waitUntil(() -> !intakeSubsystem.isDetectingCoral()))
                // .andThen(Commands.waitSeconds(IntakeConstants.kTimerThreshold)) // DO WE NEED ANOTHER THRESHOLD
                .andThen(Commands.runOnce(intakeSubsystem::stopCoralIntake));
    }

    // public Command scoreCoralAndAlgae(){
    //     return Commands.run(null, intakeSubsystem)
    // }

    public Command scoreAlgae() {
        return Commands.runOnce(intakeSubsystem::reverseAlgaeIntake);
    }

    public Command stopScoringAlgae() {
        return Commands.runOnce(intakeSubsystem::stopAlgaeIntake, intakeSubsystem);
    }

    public Command stopScoringCoral() {
        return Commands.runOnce(intakeSubsystem::stopCoralIntake, intakeSubsystem);
    }

    public Command scoreCoralContinuous() {
        return Commands.runOnce(intakeSubsystem::intakeCoral, intakeSubsystem);
    }

}

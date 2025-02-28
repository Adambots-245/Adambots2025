package com.adambots.commands;

import com.adambots.subsystems.HangSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HangCommands {

    HangSubsystem hangSubsystem;

    public HangCommands(HangSubsystem hangSubsystem) {
        this.hangSubsystem = hangSubsystem;
    }

    public Command pushOutHang() {
        return new InstantCommand(
            ()-> {
                hangSubsystem.setSolenoids(false);
                hangSubsystem.setMotorSpeed(-0.5);
            },
            hangSubsystem
        );
    }

    public Command pullInHang() {
        return new InstantCommand(
            ()-> {
                hangSubsystem.setSolenoids(true);
                hangSubsystem.setMotorSpeed(1);
            },
            hangSubsystem
        );
    }
}

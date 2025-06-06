package com.adambots.commands;

import com.adambots.Constants.HangConstants;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.subsystems.HangSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.subsystems.CANdleSubsystem.AnimationTypes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HangCommands {

    HangSubsystem hangSubsystem;
    CANdleSubsystem caNdleSubsystem;

    public HangCommands(HangSubsystem hangSubsystem, CANdleSubsystem caNdleSubsystem) {
        this.hangSubsystem = hangSubsystem;
        this.caNdleSubsystem = caNdleSubsystem;
    }

    public Command pushOutHang() {
        return Commands.either(
                new InstantCommand(
                        () -> {
                            hangSubsystem.setMotorSpeed(-1);
                        },
                        hangSubsystem),
                new InstantCommand(
                        () -> {
                            hangSubsystem.toggleSolenoids();
                            hangSubsystem.setMotorSpeed(-1);
                        },
                        hangSubsystem),
                () -> hangSubsystem.getSolenoid());

    }

    public Command pullInHang() {
        return Commands.either(
                new InstantCommand(
                        () -> {
                            hangSubsystem.toggleSolenoids();
                            hangSubsystem.setMotorSpeed(0.9);
                        },
                        hangSubsystem),
                new InstantCommand(
                        () -> {
                            hangSubsystem.setMotorSpeed(0.9);
                        },
                        hangSubsystem),
                () -> hangSubsystem.getSolenoid());
    }

    public Command autoHang() {
        return Commands.either(
                new InstantCommand(
                        () -> {
                            hangSubsystem.toggleSolenoids();
                            hangSubsystem.setMotorSpeed(1);
                        },
                        hangSubsystem).until(()->hangSubsystem.isLimitPressed()),
                new InstantCommand(
                        () -> {
                            hangSubsystem.setMotorSpeed(1);
                        },
                        hangSubsystem).until(()->hangSubsystem.isLimitPressed()),
                () -> hangSubsystem.getSolenoid());
    }
}

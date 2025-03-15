package com.adambots.commands;

import com.adambots.Constants.HangConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.HangSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HangCommands {

    HangSubsystem hangSubsystem;
    CANdleSubsystem caNdleSubsystem;

    public HangCommands(HangSubsystem hangSubsystem, CANdleSubsystem caNdleSubsystem) {
        this.hangSubsystem = hangSubsystem;
        this.caNdleSubsystem = caNdleSubsystem;
    }

    public Command pushOutHang() {
        return new InstantCommand(
            ()-> {
                hangSubsystem.toggleSolenoids();
                hangSubsystem.setMotorSpeed(-1);
            },
            hangSubsystem
        );
    }

    public Command pullInHang() {
        return new InstantCommand(
            ()-> {
                hangSubsystem.toggleSolenoids();
                hangSubsystem.setMotorSpeed(1);
            },
            hangSubsystem
        );
    }

    public Command autoHang() {
        return new InstantCommand(
            ()-> {
                hangSubsystem.setSolenoids(false);
                hangSubsystem.setMotorSpeed(1);
                caNdleSubsystem.setAnimation((AnimationTypes.Fire));
                // if(hangSubsystem.getPitch() > HangConstants.kRobotAngleStopHang){
                //     //hangSubsystem.setMotorSpeed(0);
                //     caNdleSubsystem.setAnimation(AnimationTypes.Rainbow);
                // }
            }
        );
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.LEDCommands;

import com.adambots.Constants.LEDConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AnimateCommand extends Command {

  private CANdleSubsystem candleSubsystem;
  
    /** Creates a new AnimateCommand. */
    public AnimateCommand(CANdleSubsystem candleSubsystem) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(candleSubsystem);
      this.candleSubsystem = candleSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candleSubsystem.setColor(LEDConstants.green);
    candleSubsystem.setAnimation(AnimationTypes.Fire);
    System.out.println("CANdle working");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

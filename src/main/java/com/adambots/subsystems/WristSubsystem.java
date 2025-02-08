// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.utils.StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  BaseMotor wristMotor;
  BaseAbsoluteEncoder wristEncoder;
  // State Machine components
  private final WristContext context;
  private final StateMachine<WristContext> stateMachine;
  private final StateMachine<WristContext>.State scoreState;
  private final StateMachine<WristContext>.State intakeState;
  private final StateMachine<WristContext>.State highScoreState;

  // Constants
  private static final double wristIntakePos = 0.0;
  private static final double wristScorePos = 0.0;
  private static final double wristHighScorePos = 0.0;

  private static final double wristTolerance = 2.0;

  public WristSubsystem(BaseMotor wristMotor, BaseAbsoluteEncoder wristEncoder) {
    this.wristMotor = wristMotor;
    this.wristEncoder = wristEncoder;

    // Initialize state machine
    context = new WristContext();
    stateMachine = new StateMachine<>(context);

    // Create states with trigger conditions
    scoreState = stateMachine.addState("Score",
        () -> (isAtPosition(wristScorePos)));

    intakeState = stateMachine.addState("Intake", () -> isAtPosition(wristIntakePos) &&
        (isAtPosition(wristIntakePos)));

    highScoreState = stateMachine.addState("HighScore", () -> isAtPosition(wristHighScorePos) &&
        (isAtPosition(wristIntakePos)));

    // Define transitions
    intakeState.addTransition(scoreState, ctx -> {
      ctx.wristTargetPosition = wristScorePos;
      ctx.wristMotorSpeed = 0.5;
    });

    scoreState.addTransition(intakeState, ctx -> {
      ctx.wristTargetPosition = wristIntakePos;
      ctx.wristMotorSpeed = -0.5;
    });

    scoreState.addTransition(highScoreState, ctx -> {
      ctx.wristTargetPosition = wristHighScorePos;
      ctx.wristMotorSpeed = 0.5;
    });

    intakeState.addTransition(highScoreState, ctx -> {
      ctx.wristTargetPosition = wristHighScorePos;
      ctx.wristMotorSpeed = 0.5;
    });

    highScoreState.addTransition(intakeState, ctx -> {
      ctx.wristTargetPosition = wristIntakePos;
      ctx.wristMotorSpeed = -0.5;
    });
    
    highScoreState.addTransition(scoreState, ctx -> {
      ctx.wristTargetPosition = wristHighScorePos;
      ctx.wristMotorSpeed = -0.5;
    });
  }

  private boolean isAtPosition(double position) {
    return Math.abs(wristEncoder.getAbsolutePositionDegrees() - position) < wristTolerance;
  }

  // private boolean isMovingTowards(double position) {
  // double current = armEncoder.getDistance();
  // return (current < position && context.motorSpeed > 0) ||
  // (current > position && context.motorSpeed < 0);
  // }

  @Override
  public void periodic() {
      // Update context
      context.wristCurrentPosition = wristEncoder.getAbsolutePositionDegrees();
      
      // Update state machine
      stateMachine.periodic();
      
      // Apply motor output
      wristMotor.set(context.wristMotorSpeed);
      
      // Update SmartDashboard
      SmartDashboard.putString("Current State", 
          stateMachine.getCurrentState().getName());
      SmartDashboard.putNumber("Arm Position", 
          context.wristCurrentPosition);
  }
  
  // Public methods for commanding the arm
  public void moveToGround() {
      stateMachine.requestTransition(intakeState);
  }
  
  public void moveToMid() {
      stateMachine.requestTransition(scoreState);
  }
  
  public void moveToHigh() {
      stateMachine.requestTransition(highScoreState);
  }
}
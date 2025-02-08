// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;
import com.adambots.utils.StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorTestSubsystem extends SubsystemBase {
  /** Creates a new ElevatorTestSubsystem. */
  // Hardware components
  private final BaseMotor KrakenMotor;
  private final BaseMotor BigNeoMotor;
  // State Machine components
  private final TestContext context;
  private final StateMachine<TestContext> stateMachine;
  private final StateMachine<TestContext>.State lowerState;
  private final StateMachine<TestContext>.State upperState;

  // Constants
  private static final double NeoLower = 0.0;
  private static final double NeoUpper = 45.0;
  private static final double KrakenLower = 90.0;
  private static final double KrakenUpper = 2.0;
  private static final double TOLERANCE = 2;

  public ElevatorTestSubsystem(BaseMotor KrakenMotor, BaseMotor BigNeoMotor) {
    this.KrakenMotor = KrakenMotor;
    this.BigNeoMotor = BigNeoMotor;

    // Initialize state machine
    context = new TestContext();
    stateMachine = new StateMachine<>(context);

    // Create states with trigger conditions
    lowerState = stateMachine.addState("Lower", () -> isAtPosition(NeoLower, KrakenLower) ||
        ((context.neoTargetPosition == NeoLower && context.neoMotorSpeed < 0)
            && (context.krakenTargetPosition == KrakenLower && context.krakenMotorSpeed < 0)));

    upperState = stateMachine.addState("Upper", () -> isAtPosition(NeoUpper, KrakenUpper) ||
        ((context.neoTargetPosition == NeoUpper && context.neoMotorSpeed < 0)
            && (context.krakenTargetPosition == KrakenUpper && context.krakenMotorSpeed < 0)));

    // Define transitions
    lowerState.addTransition(upperState, ctx -> {
      ctx.krakenTargetPosition = KrakenUpper;
      ctx.krakenMotorSpeed = 0.5;
      ctx.neoTargetPosition = NeoUpper;
      ctx.neoMotorSpeed = 0.5;
    });

    upperState.addTransition(lowerState, ctx -> {
      ctx.krakenTargetPosition = KrakenLower;
      ctx.krakenMotorSpeed = -0.5;
      ctx.neoTargetPosition = NeoLower;
      ctx.neoMotorSpeed = -0.5;
    });
  }

  private boolean isAtPosition(double NeoPosition, double KrakenPosition) {
    return (Math.abs(BigNeoMotor.getPosition() - NeoPosition) < TOLERANCE)
        && (Math.abs(KrakenMotor.getPosition() - KrakenPosition) < TOLERANCE);
  }

  @Override
  public void periodic() {
    // Update context
    context.neoCurrentPosition = BigNeoMotor.getPosition();
    context.krakenCurrentPosition = KrakenMotor.getPosition();

    // Update state machine
    stateMachine.periodic();

    // Apply motor output
    BigNeoMotor.set(context.neoMotorSpeed);
    KrakenMotor.set(context.krakenMotorSpeed);

    // Update SmartDashboard
    SmartDashboard.putString("Current State",
        stateMachine.getCurrentState().getName());
    SmartDashboard.putNumber("Big NEO motor position",
        context.neoCurrentPosition);
    SmartDashboard.putNumber("Kraken motor position",
        context.krakenCurrentPosition);
  }
 public void moveToUpperState() {
        stateMachine.requestTransition(upperState);
    }
    
    public void moveToLowerState() {
        stateMachine.requestTransition(lowerState);
    }
    

}


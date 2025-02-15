// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.PhotoEye;
import com.adambots.utils.StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  // Hardware
  private BaseMotor elevatorMotor;
  private PhotoEye intakeEye;
  private PhotoEye L1Eye;
  private PhotoEye L2Eye;
  private PhotoEye L3Eye;
  private PhotoEye L4Eye;

  // State Machine components
  private final ElevatorStates context;
  private final StateMachine<ElevatorStates> stateMachine;

  // States
  private final StateMachine<ElevatorStates>.State intakeState;
  private final StateMachine<ElevatorStates>.State L1state;
  private final StateMachine<ElevatorStates>.State L2state;
  private final StateMachine<ElevatorStates>.State L3state;
  private final StateMachine<ElevatorStates>.State L4state;

  // Elevator Position Constants
  private static final double ElevatorIntakePos = 0;
  private static final double ElevatorL1Pos = 0;
  private static final double ElevatorL2Pos = 0;
  private static final double ElevatorL3Pos = 0;
  private static final double ElevatorL4Pos = 0;

  private static final double TOLERANCE = 2.0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(BaseMotor elevatorMotor) {
    this.elevatorMotor = elevatorMotor;

    // Initialize state machine
    context = new ElevatorStates();
    stateMachine = new StateMachine<>(context);

    // Create states with trigger conditions
    intakeState = stateMachine.addState("Intake State", () -> {
      boolean isAtIntake = intakeEye.isDetecting() &&
          (context.elevatorTargetPosition == ElevatorIntakePos);
      if (isAtIntake) {
        elevatorMotor.setPosition(ElevatorIntakePos);
        return true;
      } else {
        return false;
      }
    });

    L1state = stateMachine.addState("L1 State", () -> {
      boolean isAtL1 = L1Eye.isDetecting() &&
          (context.elevatorTargetPosition == ElevatorIntakePos);
      if (isAtL1) {
        elevatorMotor.setPosition(ElevatorL1Pos);
        return true;
      } else {
        return false;
      }
    });

    L2state = stateMachine.addState("L2 State", () -> {
      boolean isAtL2 = L2Eye.isDetecting() &&
          (context.elevatorTargetPosition == ElevatorIntakePos);
      if (isAtL2) {
        elevatorMotor.setPosition(ElevatorL2Pos);
        return true;
      } else {
        return false;
      }
    });

    L3state = stateMachine.addState("L3 State", () -> {
      boolean isAtL3 = L3Eye.isDetecting() &&
          (context.elevatorTargetPosition == ElevatorIntakePos);
      if (isAtL3) {
        elevatorMotor.setPosition(ElevatorL3Pos);
        return true;
      } else {
        return false;
      }
    });

    L4state = stateMachine.addState("L4 State", () -> {
      boolean isAtL4 = L4Eye.isDetecting() &&
          (context.elevatorTargetPosition == ElevatorIntakePos);
      if (isAtL4) {
        elevatorMotor.setPosition(ElevatorL1Pos);
        return true;
      } else {
        return false;
      }
    });

    // Define transitions
    intakeState.addTransition(L1state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL1Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L1state.addTransition(L2state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL2Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L2state.addTransition(L3state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL3Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L3state.addTransition(L4state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL4Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L4state.addTransition(L3state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL3Pos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    L3state.addTransition(L2state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL2Pos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    L2state.addTransition(L1state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL1Pos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    L1state.addTransition(intakeState, ctx -> {
      ctx.elevatorTargetPosition = ElevatorIntakePos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    intakeState.addTransition(L2state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL2Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L2state.addTransition(intakeState, ctx -> {
      ctx.elevatorTargetPosition = ElevatorIntakePos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    intakeState.addTransition(L3state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL3Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L3state.addTransition(intakeState, ctx -> {
      ctx.elevatorTargetPosition = ElevatorIntakePos;
      ctx.elevatorMotorSpeed = -0.5;
    });

    intakeState.addTransition(L4state, ctx -> {
      ctx.elevatorTargetPosition = ElevatorL4Pos;
      ctx.elevatorMotorSpeed = 0.5;
    });

    L4state.addTransition(intakeState, ctx -> {
      ctx.elevatorTargetPosition = ElevatorIntakePos;
      ctx.elevatorMotorSpeed = -0.5;
    });
  }

  // private boolean isAtPosition(double position) {
  // return Math.abs(elevatorMotor.getPosition() - position) < TOLERANCE;
  // }

  private boolean isMovingTowards(double position) {
    double current = elevatorMotor.getPosition();
    return (current < position && context.elevatorMotorSpeed > 0) ||
        (current > position && context.elevatorMotorSpeed < 0);

  }

  @Override
  public void periodic() {
    context.elevatorPosition = elevatorMotor.getPosition();
    // Update state machine
    stateMachine.periodic();

    // Apply motor output
    elevatorMotor.set(context.elevatorMotorSpeed);

    // Update SmartDashboard
    SmartDashboard.putString("Current State",
        stateMachine.getCurrentState().getName());
    SmartDashboard.putNumber("Elevator Position",
        context.elevatorPosition);
  }

  // Public methods for commanding the arm
  public void moveToIntake() {
    stateMachine.requestTransition(intakeState);
  }

  public void moveToL1() {
    stateMachine.requestTransition(L1state);
  }

  public void moveToL2() {
    stateMachine.requestTransition(L2state);
  }

  public void moveToL3() {
    stateMachine.requestTransition(L3state);
  }

  public void moveToL4() {
    stateMachine.requestTransition(L4state);
  }
}
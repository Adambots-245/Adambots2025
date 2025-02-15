// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.adambots.subsystems;

// import java.lang.Thread.State;

// import com.adambots.actuators.BaseMotor;
// import com.adambots.sensors.BaseProximitySensor;
// import com.adambots.utils.StateMachine;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ElevatorTestSubsystem extends SubsystemBase {
//   /** Creates a new ElevatorTestSubsystem. */
//   // Hardware components
//   private final BaseMotor KrakenMotor;
//   private final BaseProximitySensor photoEyeLow;
//   private final BaseProximitySensor photoEyeUp;



//   // private final BaseMotor BigNeoMotor;
//   // State Machine components
//   private final TestContext context;
//   private final StateMachine<TestContext> stateMachine;
//   private final StateMachine<TestContext>.State lowerState;
//   private final StateMachine<TestContext>.State upperState;

//   // Constants
//   private static final double NeoLower = 0.0;
//   private static final double NeoUpper = 100.0;
//   private static final double KrakenLower = -1;
//   private static final double KrakenUpper = 1;
//   private static final double TOLERANCE = 0.3;

//   String targetStateName = "";



//   public ElevatorTestSubsystem(BaseMotor KrakenMotor, BaseProximitySensor photoEyeUp, BaseProximitySensor photoEyeLow/*, BaseMotor BigNeoMotor*/ ) {
//     this.KrakenMotor = KrakenMotor;
//     this.photoEyeLow = photoEyeLow;
//     this.photoEyeUp = photoEyeUp;
//     // this.BigNeoMotor = BigNeoMotor;

//     // Initialize state machine
//     context = new TestContext();
//     // stateMachine = new StateMachine<>(context);


//     // Create states with trigger conditions
//     lowerState = stateMachine.addState("Lower", () -> {
//       return isAtPosition(photoEyeLow); /* && (context.krakenTargetPosition == KrakenLower);
//       /* (context.neoTargetPosition == NeoUpper && context.neoMotorSpeed < 0)
//           && */
//     } );
//     upperState = stateMachine.addState("Upper", () -> { 
//       return isAtPosition(photoEyeUp); /* && (context.krakenTargetPosition == KrakenUpper);
//       /* (context.neoTargetPosition == NeoUpper && context.neoMotorSpeed < 0)
//           && */

//     } );

//     // Define transitions
//     lowerState.addTransition(upperState, ctx -> {
//       targetStateName = upperState.getName();
//       ctx.krakenTargetPosition = KrakenUpper;
//       ctx.krakenMotorSpeed = -0.02;
//       System.out.println("running to up");
//       // ctx.neoTargetPosition = NeoUpper;
//       // ctx.neoMotorSpeed = 0.5;
//     });

//     upperState.addTransition(lowerState, ctx -> {
//       targetStateName = lowerState.getName();
//       ctx.krakenTargetPosition = KrakenLower;
//       ctx.krakenMotorSpeed = 0.02;
//       System.out.println("running to low");

//       // ctx.neoTargetPosition = NeoLower;
//       // ctx.neoMotorSpeed = -0.5;
//     });
//   }

//   private boolean isAtPosition(BaseProximitySensor photoEye) {
//     return photoEye.isDetecting();
//     // return  /*(Math.abs(BigNeoMotor.getPosition() - NeoPosition) < TOLERANCE)
//     //     && */(Math.abs(KrakenMotor.getPosition() - KrakenPosition) < TOLERANCE);
//   }

  
//   @Override
//   public void periodic() {
//     // Update context
//     // context.neoCurrentPosition = BigNeoMotor.getPosition();
//     context.krakenCurrentPosition = KrakenMotor.getPosition();
//     SmartDashboard.putNumber("current pos", context.krakenCurrentPosition);
//     SmartDashboard.putNumber("target pos", context.krakenTargetPosition);

//     SmartDashboard.putNumber("error", Math.abs(KrakenMotor.getPosition() - context.krakenTargetPosition));
//     SmartDashboard.putBoolean("is at up", isAtPosition(photoEyeUp));
//     SmartDashboard.putBoolean("is at low", isAtPosition(photoEyeLow));



//     // Update state machine
//     stateMachine.periodic();

//     if (stateMachine.getCurrentState().getName().equals(targetStateName)) {
//       context.krakenMotorSpeed = 0;
//     }

//     // Apply motor output
//     // BigNeoMotor.set(context.neoMotorSpeed);
//     // if (isAtPosition(context.neoTargetPosition, context.krakenTargetPosition)) {
//     //   KrakenMotor.set(0);

//     // } else {
//     SmartDashboard.putNumber("motor speed", context.krakenMotorSpeed);
//     KrakenMotor.set(context.krakenMotorSpeed);
//     // }

//     // Update SmartDashboard
//     SmartDashboard.putString("Current State",
//         stateMachine.getCurrentState().getName());
//     SmartDashboard.putString("targetState", targetStateName);
//     // SmartDashboard.putNumber("Big NEO motor position",
//     //     context.neoCurrentPosition);
//     SmartDashboard.putNumber("Kraken motor position",
//         context.krakenCurrentPosition);
//   }
//  public void moveToUpperState() {
//         stateMachine.requestTransition(upperState);
//     }
    
//     public void moveToLowerState() {
//         stateMachine.requestTransition(lowerState);
//     }
    

// }


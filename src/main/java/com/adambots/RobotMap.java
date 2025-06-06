/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.lib.actuators.BaseActuator;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseServo;
import com.adambots.lib.actuators.BaseServo.ServoMode;
import com.adambots.lib.actuators.BaseSolenoid;
import com.adambots.lib.actuators.DirectServo;
import com.adambots.lib.actuators.ElectricalSolenoid;
import com.adambots.lib.actuators.MinionMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseAbsoluteEncoder;
import com.adambots.lib.sensors.BaseDistanceSensor;
import com.adambots.lib.sensors.BaseGyro;
import com.adambots.lib.sensors.CANRangeSensor;
import com.adambots.lib.sensors.Gyro;
import com.adambots.lib.sensors.LimitSwitch;
import com.adambots.lib.sensors.ThroughBoreEncoder;
import com.ctre.phoenix.led.CANdle;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * Define all the devices here
 */
public class RobotMap {
    // Robot Device Ports - PDP should be on port 1, and if pnematics are present, PCM must go on port 0
    public static final int kPDMPort = 1;
    public static final int kGyroPort = 6;
    public static final int kCANdlePort = 31;

    // Drive Ports
    public static final int kFrontRightTurningEncoderPort = 5;
    public static final int kFrontRightTurningMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 20;

    public static final int kRearRightTurningEncoderPort = 3;
    public static final int kRearRightTurningMotorPort = 18;
    public static final int kRearRightDriveMotorPort = 21;
    
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kRearLeftTurningMotorPort = 14;
    public static final int kRearLeftDriveMotorPort = 23;

    public static final int kFrontLeftTurningEncoderPort = 4;
    public static final int kFrontLeftTurningMotorPort = 15;
    public static final int kFrontLeftDriveMotorPort = 22;  

    // Intake Ports
    public static final int kIntakeMotorPort = 16;
    // public static final int kFirstIntakeLimPort = 3;
    // public static final int kSecondIntakeLimPort = 4;
    public static final int kTopCoralServoChannel = 3;
    public static final int kBottomCoralServoChannel = 2;
    public static final int kGripperAlgaeServoChannel = 1;
    public static final int kRunnerAlgaeServoChannel = 2;
    public static final int kCANrangePort = 60;
    public static final int kAlgaeCANrangePort = 35;
    public static final int kServoHubPort = 45;
    public static final int kMinionPort = 48;
    public static final int kMinionLeftPort = 50;
    public static final int kMinionRightPort = 51;
    public static final int kClimbMotorPort = 19;
    public static final int kClimbLimitSwitchPort = 4; 
    public static final int kLowerKrakenLimitSwitchPort = 1; 
    public static final int kUpperKrakenLimitSwitchPort = 0; 


    // public static final int kHPSrangePort = 35;

    // Operator Interface Ports (Joystick and XBoxControllers)
    public static final int kJoystickControllerPort = 0;
    public static final int kXboxControllerPort = 1;

    // Elevator Ports
    public static final int kWristMotorPort = 33;
    public static final int kEncoderPort = 3;
    public static final int kElevatorMotorPort = 32;
    


    //Robot Devices
    public static final PowerDistribution PDM = new PowerDistribution(kPDMPort, ModuleType.kRev);
    public static final BaseGyro gyro = new Gyro(kGyroPort);
    public static final CANdle candleLEDs = new CANdle(kCANdlePort);
    // public static final ServoHub hub = new ServoHub(kServoHubPort);
    // public static final BaseActuator topCoralActuator = new CRHubServo(hub, kTopCoralServoChannel);
    // public static final BaseActuator bottomCoralActuator = new CRHubServo(hub, kBottomCoralServoChannel);
    public static final BaseMotor minionMotor = new MinionMotor(kMinionPort, "*");
    public static final BaseMotor algaeLeftMotor = new MinionMotor(kMinionLeftPort, "*");
    public static final BaseMotor algaeRightMotor = new MinionMotor(kMinionRightPort, "*");

    // public static final BaseActuator algaeGripper = new DirectServo(kGripperAlgaeServoChannel, ServoMode.CONTINUOUS_ROTATION);
    // public static final BaseActuator algaeRunner = new DirectServo(kRunnerAlgaeServoChannel, ServoMode.CONTINUOUS_ROTATION);
    // public static final BaseActuator topCoralActuator = new TalonFXMotor(kIntakeMotorPort, true, 40, false);
    public static final BaseDistanceSensor coralCANrange = new CANRangeSensor(kCANrangePort, true);
    public static final BaseDistanceSensor algaeCANrange = new CANRangeSensor(kAlgaeCANrangePort, true);
    // public static final BaseDistanceSensor HPSrange = new CANRangeSensor(kHPSrangePort, true);

    public static final BaseMotor wristMotor = new TalonFXMotor(kWristMotorPort, true, 20, false);
    public static final BaseAbsoluteEncoder wristEncoder = new ThroughBoreEncoder(kEncoderPort);
    public static final BaseMotor elevatorMotor = new TalonFXMotor(kElevatorMotorPort, true, 40, true);
    public static final BaseMotor climbMotor = new TalonFXMotor(kClimbMotorPort, true, 40, true);
    public static final BaseServo climbServo = new DirectServo(0, ServoMode.ANGULAR);
    public static final BaseSolenoid climbSolenoid = new ElectricalSolenoid(0);
    public static final LimitSwitch hangLimitSwitch = new LimitSwitch(kClimbLimitSwitchPort, false);

    public static final LimitSwitch lowerElevatorLimitSwitch = new LimitSwitch(kLowerKrakenLimitSwitchPort, false);
    public static final LimitSwitch upperElevatorLimitSwitch = new LimitSwitch(kUpperKrakenLimitSwitchPort, false);
}

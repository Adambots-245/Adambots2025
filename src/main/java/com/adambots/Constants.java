package com.adambots;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;

/**
 * All constant values for robot operation - Any ports should be defined in {@link RobotMap} 
 */
public final class Constants {
    public static final String kDefaultShuffleboardTab = "debug";
    public static Field2d odomField = new Field2d();    
    public static final Boolean enableAutomaticShuffleboardRecording = false;

    public static final class LEDConstants {
        public static final int LEDS_IN_STRIP = 100;
        public static final LEDStripType LED_STRIP_TYPE = LEDStripType.GRB; // if this is not set properly, the colors will not work

        public static final Color off = new Color(0, 0, 0);
        public static final Color adambotsYellow = new Color(255, 255, 0);
        public static final Color yellow = new Color(255, 255, 0);
        public static final Color blue = new Color(0, 0, 255);
        public static final Color orange = new Color(180, 90, 5);
        public static final Color pink = new Color(255, 200, 200);
        public static final Color purple = new Color(255, 0, 255);
        public static final Color red = new Color(255, 150, 0);
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = new Color(255, 255, 255);
    }

    public static final class VisionConstants {
        public static final Pose2d aprilTagRedPose2d = new Pose2d(new Translation2d(1.23, 2.55), new Rotation2d());
        // public static final String noteLimelite = "limelight-noteii";
        // public static final String noteLimelite = "limelight-aprilgg";

        // public static final String aprilLimelite = "limelight-aprilgg";
        public static final String defaultAprilLimelite = "limelight-aprilgg";

        // public static final String defaultAprilLimelite = "limelight-gg";


        // public static final double kPThetaController = 5; 
        // public static final double kDThetaController = 0.08;

        // public static final double kPOdomThetaController = 4.7; 
        // public static final double kDOdomThetaController = 0.25;

        // public static final double kPTranslateController = 0.05; 
        // public static final double kDTranslateController = 0.0001;

        // public static final Translation2d blueTargetPoint = new Translation2d(0.0, 5.32);
        // public static final Translation2d redTargetPoint = new Translation2d(16.6, 5.32);

        public static final Translation2d aprilTagPos = new Translation2d(0.0, 5.55);

        public static final double kFieldWidth = 16.6;
    }

    public static final class DriveConstants {
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kRearRightDriveMotorReversed = true;

        public static final double kReefAllignOffset = 0.185;

        // Distance between centers of right and left wheels on robot in meters
        public static final double kTrackWidth = 0.61;
        // Distance between front and back wheels on robot in meters
        public static final double kWheelBase = 0.61;
        // Drive base radius in meters. Distance from robot center to furthest module, hypotenuse of kTrackWidth/2 and kWheelBase/2
        public static final double kDrivebaseRadius = Math.hypot(kTrackWidth/2, kWheelBase/2);

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
      
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        // Xbox controller joystick deadzone
        public static final double kDeadZone = 0.0;

        //Max speed of the robot in m/s, used in teleop and auton (should be set to real world value)
        //Drive the robot on carpet and measure the speed with a stopwatch
        //When FOC is enabled - 17.1 ft/s × 0.3048 m/ft = 5.21 m/s. When FOC is disabled - 17.7 ft/s × 0.3048 m/ft = 5.40 m/s. [ 1 foot = 0.3048 meters ]
        // Previous year's value 4.35; 
        public static final double kMaxSpeedMetersPerSecond = 5.21; 
        
        //Rotational speed factor in rad/s of the robot to be used for the teleop drive command
        public static final double kTeleopRotationalSpeed = 10; 
    }

    public static final class ModuleConstants {
        public static final double kMK4L1GearRatio = 1/8.14; // 1:8.14 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909
        public static final double kMK4L2GearRatio = 1/6.75; // 1:6.75 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909
        public static final double kMK4L3GearRatio = 1/6.12; // 1:6.12 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909

        // The 2025 Robot will use an Adapter Kit to work with Kraken. This changes the Gear Ratios to L1+, L2+ and L3+
        public static final double kMK4IL1PlusGearRatio = 1/7.13; // 1:7.13 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957
        public static final double kMK4IL2PlusGearRatio = 1/5.9; // 1:5.9 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957
        public static final double kMK4IL3PlusGearRatio = 1/5.36; // 1:5.36 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957

        //Define gear ratio as motor revolutions per wheel rotation
        public static final double kSteeringGearRatio = 150.0/7.0; // 150/7:1 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909        

        public static final int kDriveCurrentLimit = 32; //Current limit in amps of drive motors, higher values mean faster acceleration but lower battery life
        public static final int kTurningCurrentLimit = 21; //Current limit in amps of turning motors
        public static final double kNominalVoltage = 12.6; //Nominal battery voltage for motor voltage compensation

        public static final double kWheelRadiusMeters = 0.0478; //0.047625 //Should be as precise as you can get it
        public static final double kSwerveModuleFinalGearRatio = kMK4IL2PlusGearRatio; //Google the swerve module model to find this value

        // Convert drive motor rpm to linear wheel speed                  Motor RPM to Wheel RPM -> RPM to rad/s -> Wheel rad/s to linear m/s 
        public static final double kDriveEncoderVelocityConversionFactor = kSwerveModuleFinalGearRatio * (Math.PI/30) * kWheelRadiusMeters;

        // Convert drive motor rotations to linear distance             Motor rot to Wheel rot -> Wheel rot to linear meters (circumference)
        public static final double kDriveEncoderPositionConversionFactor = kSwerveModuleFinalGearRatio * 2*Math.PI * kWheelRadiusMeters;

        public static final double kPModuleTurningController = 0.7; //PID Values for turning motors
        // public static final double kDModuleTurningController = 0.026;
        public static final double kDModuleTurningController = 0;
    }

    public static final class AutoConstants {
        // PD values for auton X, Y translational movement
        // public static final double kPTranslationController = 5; 
        // public static final double kITranslationController = 0; 
        // public static final double kDTranslationController = 0.11;

        public static final double kPTranslationController = 3; 
        public static final double kITranslationController = 0; 
        public static final double kDTranslationController = 0.05;

        // PD values for auton rotational movement
        public static final double kPThetaController = 1.5; 
        public static final double kIThetaController = 0; 
        public static final double kDThetaController = 0.06;

        public static final double kPWaypointTranslation = 5; 
        public static final double kDWaypointTranslation = 0.55;

        public static final double kMaxWaypointTranslateSpeed = 2;
        public static final double kMinWaypointTranslateSpeed = 0.7;
        
        public static final Pose2d S1_POSE2D = new Pose2d(new Translation2d(1.33, 6.76), new Rotation2d(Math.toRadians(44)));
        public static final Pose2d S2_POSE2D = new Pose2d(new Translation2d(1.40, 5.45), new Rotation2d(Math.toRadians(0)));
        public static final Pose2d S3_POSE2D = new Pose2d(new Translation2d(1.33, 4.26), new Rotation2d(Math.toRadians(-44)));
    }

    // Subystem Constants go here
    public final static class IntakeConstants {
        public static final double kMaxSpeed = -1;
        public static final double kLowSpeed = .5;
        public static final double kReverseSpeed = 1;
        public static final double kDistanceToDetect = 10;//cm
        public static final double kTimerThreshold = 0.4; // number of seconds to wait before stopping the intake after detecting a coral
        public static final double kAlgaeIntakePulseSeconds = 0.2;

    }

    public final static class ElevatorConstants {

        // PID Constants
        public static final int kPIDSlot = 0;
        public static final double kElevatorSpeed = 0.2; // only for free movement and not for positional control
        public static final double kElevatorPositionIncrement = 0.7;
        public static final double kElevatorPositionTolerance = 0.5;
        public static final double kWristPositionTolerance = 2.0;
        
        public static final double kPElevatorController = 0.5;
        public static final double kIElevatorController = 0.000;
        public static final double kDElevatorController = 0.01;
        public static final double kFElevatorController = 0.0;

        public static final double kWristPositionIncrement = 3;
        public static final double kPWristController = 0.022;
        public static final double kIWristController = 0.00;
        public static final double kDWristController = 0.0001;

        // Elevator Positions
        public static final double kElevatorIntakePosition = 0.0;
        public static final double kElevatorL1Position = 21.01;
        public static final double kElevatorL2Position = 32.3;
        public static final double kElevatorL3Position = 55;
        public static final double kElevatorL4Position = 91;
        public static final double kElevatorHighAlgaePosition = 61;
        public static final double kElevatorLowAlgaePosition = 32;
        public static final double kElevatorProcessorPosition = 0;

        public static final double kElevatorMaxHeight = 90;
        public static final double kElevatorMinHeight = 0;
        public static final double kElevatorDangerZoneStart = 20;
        public static final double kElevatorDangerZoneEnd = 40;

        // Wrist Positions
        public static final double kWristStowedPosition = 55;
        public static final double kWristIntakePosition = 77;
        public static final double kWristL1Position = 66;
        public static final double kWristL2Position = 66;
        public static final double kWristL3Position = 66;
        public static final double kWristL4Position = 74;
        public static final double kWristHighAlgaePosition = 12;
        public static final double kWristLowAlgaePosition = 12;
        public static final double kWristProcessorPosition = 12;
        public static final double kWristGroundIntakePosition = 350;
        public static final double kWristHangPosition = 80;


        public static final double kWristMinAngle = 355;
        public static final double kWristMaxAngle = 85;
        public static final double kWristDangerZoneAngle = 58;

        // Gear Ratio and Other Elevator Constants
        public static final double stateChangeDelay = 1;
        public static final double kGearRatio = 10/1.0; //10:1
        public static final double kDrumCircumference = 2.0; // inches
        public static final double kInchesPerRotation = kDrumCircumference / kGearRatio;
        }
    // Command Constants go here
}
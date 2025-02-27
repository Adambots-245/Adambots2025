package com.adambots.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;
import java.util.function.Supplier;

import com.adambots.Robot;
import com.adambots.subsystems.SwerveSubsystem;

public class DriveToPoseReefAdvanced extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private int[] tagIds;
    private double offset = 0.162;
    private Supplier<Integer> aprilTagId;

    private boolean isSeen;
    private int idSeen = -1;
    private boolean isOffsetRight;
    private boolean isInveresed;
    private int reefAngle;
    private boolean isCalculated;
    private Pose2d targetPose;

    private final HolonomicDriveController holonomicController;

    public DriveToPoseReefAdvanced(SwerveSubsystem swerveSubsystem, boolean isRight) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        // this.aprilTagId = aprilTagId;
        this.isOffsetRight = isRight;

        // PID Controllers for X, Y movement & theta (rotation)
        PIDController xController = new PIDController(1.0, 0, 0);
        PIDController yController = new PIDController(1.0, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                3.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.holonomicController = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void initialize() {
        isSeen = false;
        isCalculated = false;
        targetPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(0)));

        if (Robot.isOnRedAlliance()) {
            tagIds = new int[]{6,7,8,9,10,11};
        } else {
            tagIds = new int[]{17,18,19,20,21,22};
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();

        idSeen = swerveSubsystem.getVision().hasID(tagIds);
        // idSeen = aprilTagId.get();

        if (idSeen > -1) {
            isSeen = true;
        }

        if (isSeen == true && isCalculated == false) {
            System.out.println(idSeen);
            switch (idSeen) {
                case 6:
                    System.out.println("CASE 6");
                    targetPose = new Pose2d(new Translation2d(13.728, 2.884), new Rotation2d(Math.toRadians(120)));
                    reefAngle = 30;
                    isInveresed = true;
                    break;
                case 7:
                    System.out.println("CASE 7");
                    targetPose = new Pose2d(new Translation2d(14.397, 4.025), new Rotation2d(Math.toRadians(180)));
                    reefAngle = 90;
                    isInveresed = true;
                    break;
                case 8:
                    System.out.println("CASE 8");
                    targetPose = new Pose2d(new Translation2d(13.718, 5.173), new Rotation2d(Math.toRadians(-120)));
                    reefAngle = 150;
                    isInveresed = true;
                    break;
                case 9:
                    System.out.println("CASE 9");
                    targetPose = new Pose2d(new Translation2d(12.401, 5.172), new Rotation2d(Math.toRadians(-60)));
                    reefAngle = 30;
                    isInveresed = false;
                    break;
                case 10:
                    targetPose = new Pose2d(new Translation2d(11.749, 4.025), new Rotation2d(0));
                    reefAngle = 90;
                    isInveresed = false;
                    break;
                case 11:
                    targetPose = new Pose2d(new Translation2d(12.412, 2.884), new Rotation2d(Math.toRadians(60)));
                    reefAngle = 150;
                    isInveresed = false;
                    break;
                case 17:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
                case 18:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
                case 19:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
                case 20:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
                case 21:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
                case 22:
                    targetPose = new Pose2d(new Translation2d(), new Rotation2d());
                    break;
            }

            if (isOffsetRight) {
                if (isInveresed) {
                    targetPose = new Pose2d(
                            new Translation2d(targetPose.getX() + offset * Math.cos(Math.toRadians(reefAngle)),
                                    targetPose.getY() + offset * Math.sin(Math.toRadians(reefAngle))),
                            targetPose.getRotation());
                } else {
                    targetPose = new Pose2d(
                            new Translation2d(targetPose.getX() - offset * Math.cos(Math.toRadians(reefAngle)),
                                    targetPose.getY() - offset * Math.sin(Math.toRadians(reefAngle))),
                            targetPose.getRotation());
                }
            } else {
                if (isInveresed) {
                    targetPose = new Pose2d(
                            new Translation2d(targetPose.getX() - offset * Math.cos(Math.toRadians(reefAngle)),
                                    targetPose.getY() - offset * Math.sin(Math.toRadians(reefAngle))),
                            targetPose.getRotation());
                } else {
                    targetPose = new Pose2d(
                            new Translation2d(targetPose.getX() + offset * Math.cos(Math.toRadians(reefAngle)),
                                    targetPose.getY() + offset * Math.sin(Math.toRadians(reefAngle))),
                            targetPose.getRotation());
                }
            }

            isCalculated = true;
        }

        if (isCalculated) {
            ChassisSpeeds targetSpeeds = holonomicController.calculate(
                    currentPose,
                    targetPose,
                    0,
                    targetPose.getRotation() // Final desired rotation
            );

            swerveSubsystem.drive(targetSpeeds); // Implement this in DriveSubsystem
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}

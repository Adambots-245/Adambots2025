package com.adambots.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;
import java.util.function.Supplier;

import com.adambots.subsystems.SwerveSubsystem;

public class DriveToPose extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> targetPoseSupplier;

    private final HolonomicDriveController holonomicController;

    public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        // PID Controllers for X, Y movement & theta (rotation)
        PIDController xController = new PIDController(2.0, 0, 0);
        PIDController yController = new PIDController(2.0, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                3.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.holonomicController = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        ChassisSpeeds targetSpeeds = holonomicController.calculate(
                currentPose, 
                targetPose, 
                0, 
                targetPose.getRotation() // Final desired rotation
        );

        swerveSubsystem.drive(targetSpeeds); // Implement this in DriveSubsystem
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0,0,0));
    }
}

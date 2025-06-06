package com.adambots.commands.driveCommands;

import com.adambots.Robot;
import com.adambots.RobotContainer;
import com.adambots.lib.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngleCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    // private CANdleSubsystem caNdleSubsystem;
    private PIDController angleTurningPIDController = new PIDController(3, 0, 0.02);
    private double drive_output;
    private double targetAngleRad;

    private boolean isAutoHumanPlayer;

    public RotateToAngleCommand(SwerveSubsystem swerveSubsystem, double targetAngleDeg, boolean isAutoHumanPlayer) {
        addRequirements(swerveSubsystem);

        angleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.swerveSubsystem = swerveSubsystem;
        this.targetAngleRad = Math.toRadians(targetAngleDeg);
        this.isAutoHumanPlayer = isAutoHumanPlayer;
    }

    @Override
    public void initialize() {
        swerveSubsystem.getVision().disableAllCameras();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Calculates the drive rotation
        if (isAutoHumanPlayer){
            if (Robot.isOnRedAlliance()){
                if (swerveSubsystem.getPose().getY() > 4){
                    targetAngleRad = Math.toRadians(-125);
                } else {
                    targetAngleRad = Math.toRadians(125);
                }
            } else {
                if (swerveSubsystem.getPose().getY() > 4){
                    targetAngleRad = Math.toRadians(125);
                } else {
                    targetAngleRad = Math.toRadians(-125);
                }
            }
        }
        if (Robot.isOnRedAlliance()) {
            drive_output = angleTurningPIDController.calculate(swerveSubsystem.getHeading().getRadians(),targetAngleRad);
        } else {
            drive_output = angleTurningPIDController.calculate(swerveSubsystem.getHeading().getRadians(), Math.PI+targetAngleRad);
        }

        // Moves left or right depending on the angle
        swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(RobotContainer.getDriveAngularVelocity().get().vxMetersPerSecond,
                        RobotContainer.getDriveAngularVelocity().get().vyMetersPerSecond, drive_output),
                swerveSubsystem.getHeading()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.getVision().enableAllCameras();             
        // driveTrainSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

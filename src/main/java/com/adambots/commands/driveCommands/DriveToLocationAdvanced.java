package com.adambots.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

import com.adambots.Robot;
import com.adambots.RobotContainer;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;
import com.adambots.vision.PhotonVision;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class DriveToLocationAdvanced extends Command {
    public enum AlignLocation {
        RIGHT_POLE, LEFT_POLE, MIDDLE_ALGAE, HUMAN_PLAYER_RIGHT, HUMAN_PLAYER_LEFT, REEF_ANGLE, BARGE_LEFT,
        BARGE_MIDDLE, BARGE_RIGHT
    }

    // Subsystem references.
    private SwerveSubsystem swerveSubsystem;
    private CANdleSubsystem caNdleSubsystem;

    // Offsets used for different alignment strategies.
    private double reefOffset = 0.18; // Offset for aligning to the reef/april tag pole.
    private double humanPlayerOffset = 0; // Offset for aligning to the human player station.  0.6
    private double robotReefOffset = 0.455; // Offset for positioning the robot relative to the tag.
    private double bargeXOffset = 0.07; 
    private double bargeYOffset = 1.15; 

    // Arrays holding the AprilTag IDs for different field elements.
    private int[] reefTagIds; // AprilTag IDs for the reef (pole) targets.
    private int[] humanPlayerTagIds; // AprilTag IDs for the human player targets.
    private int[] bargeTagIds;
    // Supplier to simulate tag IDs during simulation.
    private Supplier<Integer> aprilTagSupplierSim;
    // Determines which alignment mode to use.

    private AlignLocation alignLocation;

    // Variables to track vision status and computed target pose.
    private boolean isSeen; // Flag indicating if an AprilTag was detected.
    private int idSeen = -1; // Detected AprilTag ID.
    private boolean isCalculated; // Flag indicating if the target pose has been calculated.
    private Pose2d targetPose; // Calculated target pose based on the detected tag.

    // PathPlanner holonomic drive controller for computing chassis speeds toward a
    // trajectory state.
    private PPHolonomicDriveController driveController;
    // PID controller used for angle turning when using alignLocation 5.
    private PIDController angleTurningPIDController = new PIDController(3, 0, 0.0);

    private Pose2d currentPose;

    /**
     * Constructor for DriveToLocationAdvanced.
     *
     * @param swerveSubsystem  The swerve drive subsystem.
     * @param aprilTagSupplier Supplier for simulating AprilTag IDs in simulation.
     * @param alignLocation    Determines alignment strategy:
     *                         0 - right pole, 1 - left pole, 2 - algae pole,
     *                         3 - human player right, 4 - human player left, 5 -
     *                         reef angle turning.
     * @param caNdleSubsystem  The CANdle subsystem for LED feedback.
     */
    public DriveToLocationAdvanced(SwerveSubsystem swerveSubsystem, Supplier<Integer> aprilTagSupplier,
            AlignLocation alignLocation, CANdleSubsystem caNdleSubsystem) {
        // Require the swerve subsystem for this command.
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.caNdleSubsystem = caNdleSubsystem;
        this.alignLocation = alignLocation;
        this.aprilTagSupplierSim = aprilTagSupplier;

        // Create PID constants for the drive controller tuning.
        PIDConstants translationConstants = new PIDConstants(2.0, 0.0, 0.0);
        PIDConstants rotationConstants = new PIDConstants(3.0, 0.0, 0.0);
        // Set the update period to 20ms (typical loop time).
        double period = 0.02;
        // Instantiate the PPHolonomicDriveController with the defined PID constants.
        this.driveController = new PPHolonomicDriveController(translationConstants, rotationConstants, period);
    }

    /**
     * Initializes command variables and selects the correct AprilTag IDs based on
     * alliance.
     */
    @Override
    public void initialize() {
        isSeen = false;
        isCalculated = false;
        // Initialize the target pose to a default value; will be updated when a tag is
        // detected.
        targetPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

        // Set the appropriate AprilTag IDs based on the alliance color.
        if (Robot.isOnRedAlliance()) {
            reefTagIds = new int[] { 6, 7, 8, 9, 10, 11 };
            humanPlayerTagIds = new int[] { 1, 2 };
            bargeTagIds = new int[] { 15 };
        } else {
            reefTagIds = new int[] { 17, 18, 19, 20, 21, 22 };
            humanPlayerTagIds = new int[] { 12, 13 };
            bargeTagIds = new int[] { 14 };
        }
    }

    /**
     * Main execution loop for the command.
     * Processes vision data, calculates the target pose, and commands the
     * drivetrain.
     */
    @Override
    public void execute() {
        // Retrieve the robot's current pose from the swerve subsystem.
        currentPose = swerveSubsystem.getPose();

        // Determine the detected AprilTag ID.
        // Use the simulated supplier if running in simulation; otherwise, use vision.
        if (Robot.isSimulation()) {
            idSeen = aprilTagSupplierSim.get();
        } else {
            // For human player alignment (alignLocation 3 or 4), use humanPlayerTagIds.
            // Otherwise, use reefTagIds.
            if (alignLocation == AlignLocation.HUMAN_PLAYER_LEFT || alignLocation == AlignLocation.HUMAN_PLAYER_RIGHT) {
                idSeen = swerveSubsystem.getVision().hasID(humanPlayerTagIds);
            } else if (alignLocation == AlignLocation.BARGE_LEFT || alignLocation == AlignLocation.BARGE_MIDDLE
                    || alignLocation == AlignLocation.BARGE_RIGHT) {
                idSeen = swerveSubsystem.getVision().hasID(bargeTagIds);
            } else {
                idSeen = swerveSubsystem.getVision().hasID(reefTagIds);
            }
        }

        // If a valid tag is detected (idSeen greater than -1), set the isSeen flag.
        if (idSeen > -1) {
            isSeen = true;
        }

        // If a tag has been detected and we haven't yet calculated a target pose:
        if (isSeen && !isCalculated) {
            // Calculate the target pose using PhotonVision, adjusted by the desired
            // transform.
            if (alignLocation == AlignLocation.RIGHT_POLE || alignLocation == AlignLocation.REEF_ANGLE) {
                // Align to the right pole: apply a positive reef offset.
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(robotReefOffset, reefOffset, new Rotation2d(Math.toRadians(180))));
            } else if (alignLocation == AlignLocation.LEFT_POLE) {
                // Align to the left pole: apply a negative reef offset.
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(robotReefOffset, -reefOffset, new Rotation2d(Math.toRadians(180))));
            } else if (alignLocation == AlignLocation.MIDDLE_ALGAE) {
                // Align to the middle (algae pole): no lateral offset.
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(0.6, 0, new Rotation2d(Math.toRadians(180))));
            } else if (alignLocation == AlignLocation.HUMAN_PLAYER_RIGHT) {
                // Align to the human player on the right:
                // Apply a negative human player offset and rotate 180°.
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(robotReefOffset, -humanPlayerOffset, new Rotation2d(Math.toRadians(0))));
            } else if (alignLocation == AlignLocation.HUMAN_PLAYER_LEFT) {
                // Align to the human player on the left:
                // Apply a positive human player offset and rotate 180°.
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(robotReefOffset, humanPlayerOffset, new Rotation2d(Math.toRadians(0))));
            } else if (alignLocation == AlignLocation.BARGE_MIDDLE) {
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(-bargeXOffset, 0, new Rotation2d(Math.toRadians(90))));
            } else if (alignLocation == AlignLocation.BARGE_LEFT) {
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(-bargeXOffset, -bargeYOffset, new Rotation2d(Math.toRadians(90))));
            } else if (alignLocation == AlignLocation.BARGE_RIGHT) {
                targetPose = PhotonVision.getAprilTagPose(idSeen,
                        new Transform2d(-bargeXOffset, bargeYOffset, new Rotation2d(Math.toRadians(90))));
            }
            // Mark that the target pose has been calculated.
            isCalculated = true;
        }

        // If the target pose has been calculated, command the drivetrain.
        if (isCalculated) {
            // Special handling for alignLocation 5: use a PID controller to rotate to the
            // target angle
            // while allowing the driver to control translation.
            // System.out.println("X " + targetPose.getX() + "Y: " + targetPose.getY());
            if (alignLocation == AlignLocation.REEF_ANGLE) {
                // Set LED color to orange to indicate angle alignment mode.
                caNdleSubsystem.setColor(Color.kOrange);
                // Compute the output from the angle turning PID controller using the current
                // heading and target rotation.
                double drive_output = angleTurningPIDController.calculate(
                        swerveSubsystem.getHeading().getRadians(),
                        targetPose.getRotation().getRadians());
                // Set LED color to blue after computing output.
                caNdleSubsystem.setColor(Color.kBlue);
                // Drive using the driver's translational inputs while applying the
                // PID-controlled rotation.
                swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                RobotContainer.getDriveAngularVelocity().get().vxMetersPerSecond,
                                RobotContainer.getDriveAngularVelocity().get().vyMetersPerSecond,
                                drive_output),
                        swerveSubsystem.getHeading()));
            } else {
                // For alignment modes other than 5, use the PathPlanner controller.
                // Create a dummy trajectory state with the target pose.
                PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
                targetState.pose = targetPose;
                targetState.heading = targetPose.getRotation();

                // Calculate the desired chassis speeds to drive from the current pose to the
                // target state.
                ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(
                        currentPose,
                        targetState);

                // Command the swerve subsystem to drive at the calculated speeds.
                swerveSubsystem.drive(targetSpeeds);

                // Provide visual feedback via LEDs:
                // If the robot is within 2 centimeters of the target position, set LED to
                // green.
                // Otherwise, set LED to red.
                // if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.10) {
                //     scoringCommands.scoreCoral();
                // }

                if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.07) {
                    caNdleSubsystem.setColor(Color.kGreen);
                } else {
                    caNdleSubsystem.setColor(Color.kRed);
                }
            }
        } else {
            // If no AprilTag is seen, default to normal manual driving.
            caNdleSubsystem.setColor(Color.kBlue);
            swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            RobotContainer.getDriveAngularVelocity().get().vxMetersPerSecond,
                            RobotContainer.getDriveAngularVelocity().get().vyMetersPerSecond,
                            RobotContainer.getDriveAngularVelocity().get().omegaRadiansPerSecond),
                    swerveSubsystem.getHeading()));
        }
    }

    /**
     * This command does not finish on its own.
     *
     * @return false Always returns false.
     */
    @Override
    public boolean isFinished() {
        // if (DriverStation.isAutonomous()){
        return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.07;
        // }
        // return false;
    }

    /**
     * Called when the command ends or is interrupted.
     * Stops the robot and sets the LED animation.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Stop robot movement by setting chassis speeds to zero.
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        // Set the LED animation to a predefined pattern (Larson animation).
        caNdleSubsystem.setAnimation(AnimationTypes.Larson);
    }
}

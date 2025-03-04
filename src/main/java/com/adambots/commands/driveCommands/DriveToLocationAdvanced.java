package com.adambots.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

import com.adambots.Robot;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.SwerveSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;
import com.adambots.vision.PhotonVision;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class DriveToLocationAdvanced extends Command {
    // Subsystem references.
    private SwerveSubsystem swerveSubsystem;
    private CANdleSubsystem caNdleSubsystem;

    // Offsets used for different alignment strategies.
    private double reefOffset = 0.18;           // Offset for aligning to the reef/april tag pole.
    private double humanPlayerOffset = 0.6;       // Offset for aligning to the human player station.
    private double robotOffset = 0.455;           // Offset for positioning the robot relative to the tag.

    // Arrays holding the AprilTag IDs for different field elements.
    private int[] reefTagIds;                   // AprilTag IDs for the reef (pole) targets.
    private int[] humanPlayerTagIds;            // AprilTag IDs for the human player targets.
    // Supplier to simulate tag IDs during simulation.
    private Supplier<Integer> aprilTagSupplierSim;
    // Determines which alignment mode to use.
    private int alignLocation;

    // Variables to track vision status and computed target pose.
    private boolean isSeen;                     // Flag if a tag is detected.
    private int idSeen = -1;                    // Detected tag ID.
    private boolean isCalculated;               // Flag if the target pose has been calculated.
    private Pose2d targetPose;                  // Target pose calculated from the detected tag.

    // PathPlanner's holonomic drive controller for computing chassis speeds.
    private final PPHolonomicDriveController driveController;

    /**
     * Constructor for DriveToLocationAdvanced.
     * 
     * @param swerveSubsystem       The swerve drive subsystem.
     * @param aprilTagSupplier      Supplier for simulating AprilTag IDs in simulation.
     * @param alignLocation         Determines alignment strategy:
     *                              0 - right pole, 1 - left pole, 2 - algae pole,
     *                              3 - human player right, 4 - human player left.
     * @param caNdleSubsystem       The CANdle subsystem for LED feedback.
     */
    public DriveToLocationAdvanced(SwerveSubsystem swerveSubsystem, Supplier<Integer> aprilTagSupplier,
            int alignLocation, CANdleSubsystem caNdleSubsystem) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.caNdleSubsystem = caNdleSubsystem;
        this.alignLocation = alignLocation;
        this.aprilTagSupplierSim = aprilTagSupplier;

        // Create PID constants for the drive controller.
        // These constants tune translation (X, Y) and rotation control.
        PIDConstants translationConstants = new PIDConstants(2.0, 0.0, 0.0);
        PIDConstants rotationConstants = new PIDConstants(3.0, 0.0, 0.0);
        // Set the update period (20ms typical loop time).
        double period = 0.02;
        // Instantiate the PPHolonomicDriveController with the PID constants and period.
        this.driveController = new PPHolonomicDriveController(translationConstants, rotationConstants, period);
    }

    /**
     * Initializes command variables and selects the correct AprilTag IDs based on alliance.
     */
    @Override
    public void initialize() {
        isSeen = false;
        isCalculated = false;
        // Default target pose. This will be replaced when a tag is detected.
        targetPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

        // Set the appropriate AprilTag IDs based on the alliance color.
        if (Robot.isOnRedAlliance()) {
            reefTagIds = new int[] { 6, 7, 8, 9, 10, 11 };
            humanPlayerTagIds = new int[] { 1, 2 };
        } else {
            reefTagIds = new int[] { 17, 18, 19, 20, 21, 22 };
            humanPlayerTagIds = new int[] { 12, 13 };
        }
    }

    /**
     * The main execution loop for the command.
     */
    @Override
    public void execute() {
        // Get the current pose of the robot from the swerve subsystem.
        Pose2d currentPose = swerveSubsystem.getPose();

        // Obtain the detected AprilTag ID using vision.
        // Use the simulated supplier in simulation; otherwise, check real vision.
        if (Robot.isSimulation()) {
            idSeen = aprilTagSupplierSim.get();
        } else {
            // For human player alignment (locations 3 & 4), use humanPlayerTagIds.
            // Otherwise, use reefTagIds.
            if (alignLocation == 3 || alignLocation == 4) {
                idSeen = swerveSubsystem.getVision().hasID(humanPlayerTagIds);
            } else {
                idSeen = swerveSubsystem.getVision().hasID(reefTagIds);
            }
        }

        // If a valid tag is detected (idSeen > -1), update flag.
        if (idSeen > -1) {
            isSeen = true;
        }

        // If a tag has been seen and we haven't calculated the target pose yet:
        if (isSeen && !isCalculated) {
            // Calculate the target pose based on the alignment mode.
            // PhotonVision.getAprilTagPose() returns a pose adjusted by the specified transform.
            if (alignLocation == 0) {
                // Align to the right pole: offset to the right (reefOffset) from the tag.
                targetPose = PhotonVision.getAprilTagPose(idSeen, new Transform2d(robotOffset, reefOffset, new Rotation2d()));
            } else if (alignLocation == 1) {
                // Align to the left pole: offset to the left (-reefOffset).
                targetPose = PhotonVision.getAprilTagPose(idSeen, new Transform2d(robotOffset, -reefOffset, new Rotation2d()));
            } else if (alignLocation == 2) {
                // Align to the middle (algae pole): no lateral offset.
                targetPose = PhotonVision.getAprilTagPose(idSeen, new Transform2d(robotOffset, 0, new Rotation2d()));
            } else if (alignLocation == 3) {
                // Align to the human player: offset to the right (-humanPlayerOffset) with a 180° rotation.
                targetPose = PhotonVision.getAprilTagPose(idSeen, new Transform2d(robotOffset, -humanPlayerOffset, new Rotation2d(Math.toRadians(180))));
            } else if (alignLocation == 4) {
                // Align to the human player: offset to the left (humanPlayerOffset) with a 180° rotation.
                targetPose = PhotonVision.getAprilTagPose(idSeen, new Transform2d(robotOffset, humanPlayerOffset, new Rotation2d(Math.toRadians(180))));
            }
            // Mark that the target pose has been calculated.
            isCalculated = true;
        }

        // If a target pose is calculated, drive toward it.
        if (isCalculated) {
            // Create a dummy trajectory state using the target pose.
            // Here, time, velocity, and acceleration are set to defaults (0) since this is a static target.
            PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
            targetState.pose = targetPose;
            targetState.heading = targetPose.getRotation();

            // Calculate the desired chassis speeds to move from the current pose to the target state.
            ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(
                    currentPose,
                    targetState);

            // Command the swerve subsystem to drive with the calculated speeds.
            swerveSubsystem.drive(targetSpeeds);

            // Provide visual feedback via CANdle LEDs:
            // If the robot is within 2 centimeters of the target translation, set the LED to green.
            // Otherwise, set the LED to red.
            if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.02) {
                caNdleSubsystem.setColor(Color.kGreen);
            } else {
                caNdleSubsystem.setColor(Color.kRed);
            }
        }
    }

    /**
     * This command does not finish on its own.
     * 
     * @return false Always returns false.
     */
    @Override
    public boolean isFinished() {
        return false;
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
        // Set the LED animation to a predefined pattern.
        caNdleSubsystem.setAnimation(AnimationTypes.Larson);
    }
}

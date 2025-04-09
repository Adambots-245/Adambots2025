// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.Robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Modified from Ironclad 2024's Vision class.
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class PhotonVision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link PhotonVision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;
  int counter = 0;

  private static boolean humanPlayerFlag = false;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public PhotonVision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    // System.out.println("CHeckpoint 0");
    for (Cameras camera : Cameras.values()) {
      Cameras.updatedCache = false;
      // System.out.println("Checkpoints 1");
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst != null && poseEst.isPresent()) {
        var pose = poseEst.get();
        // counter++;
        // if (counter > 25) {
        // System.out.println(pose.estimatedPose.getX() + " Y " +
        // pose.estimatedPose.getY());
        // counter = 0;
        // }
        // System.out.println(pose.estimatedPose.getX() + "Y " +
        // pose.estimatedPose.getY());
        // System.err.println("Checkpoint 4");
        // System.out.println(camera);
        if (!humanPlayerFlag && camera != Cameras.CENTER_CAM){
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
          pose.timestampSeconds,
          camera.curStdDevs);
          // System.out.println("REEF UPDATE for " + camera.name());
        }
        
        if (camera == Cameras.CENTER_CAM && humanPlayerFlag){
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
          pose.timestampSeconds,
          camera.curStdDevs);
          // System.out.println("CENTER UPDATE for " + camera.name());
        }
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    // System.out.println("Checkpoint 1");
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the
   * camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  @Deprecated(since = "2024", forRemoval = true)
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
    if (pose.isPresent()) {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed) {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
          bestTargetAmbiguity = ambiguity;
        }
      }
      // ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity) {
        return Optional.empty();
      }

      // est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
        longDistangePoseEstimationCount++;

        // if it calculates that were 10 meter away for more than 10 times in a row its
        // probably right
        if (longDistangePoseEstimationCount < 10) {
          return Optional.empty();
        }
      } else {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  public Transform2d getDistanceFromAprilTagX(int id) {
    Pose2d aprilTagPose = getAprilTagPose(id, new Transform2d());
    return aprilTagPose.minus(currentPose.get());
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  public int hasID(int[] tagIDs) {
    for (Cameras camera : Cameras.values()) {
      for (PhotonPipelineResult result : camera.resultsList) {
        // System.out.println("RESULTS " + result);

        if (result.hasTargets()) {
          // System.out.println("HAS TARGETS " + camera.name());

          for (PhotonTrackedTarget i : result.getTargets()) {
            for (int id : tagIDs) {
              if (i.getFiducialId() == id) {
                return i.getFiducialId();
              }
            }
          }
        }
      }
    }
    return -1;
  }

  public static List<Integer> getAllDetectedTags() {
    List<Integer> detectedTagIDs = new ArrayList<>();

    for (Cameras camera : Cameras.values()) {
        for (PhotonPipelineResult result : camera.resultsList) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    detectedTagIDs.add(target.getFiducialId());
                }
            }
        }
    }

    return detectedTagIDs;
}
  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    // if (Desktop.isDesktopSupported() &&
    // Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
    // try
    // {
    // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
    // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
    // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
    // } catch (IOException | URISyntaxException e)
    // {
    // e.printStackTrace();
    // }
    // }
  }

  public boolean hasTarget() {
    // boolean isTarget = false;
    // for (Cameras c : Cameras.values()){
    // c.clearCache();
    // c.updateUnreadResults();
    // if (!c.getLatestResult().isEmpty()){
    // isTarget = true;
    // }
    // isTarget = Cameras.updatedCache;
    // }
    return Cameras.updatedCache;
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
    System.out.println(Cameras.CENTER_CAM.allowedTagIDs[0]);
  }

  /**
   * Camera Enum to select each camera
   * Google Search for WPILib Coorinate System for more information -
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
   * 
   * Rotation 3D:
   * Roll: Side-to-side tilt (rarely used, usually 0)
   * Pitch: Up/down angle
   * 
   * Use a digital angle finder or phone app
   * Measure angle between horizontal and camera's forward view
   * Positive = camera tilted up
   * 
   * 
   * Yaw: Left/right angle from robot's forward direction
   * 
   * Positive = camera rotated left
   * Negative = camera rotated right
   * 
   * Translation 3D:
   * Define robot center (typically center of rotation between wheels)
   * Measure with a tape measure/ruler:
   * 
   * X: Distance forward(+) or back(-) from robot center to camera lens
   * Y: Distance left(+) or right(-) from robot center to camera lens
   * Z: Height from floor to camera lens
   * 
   * VecBuilder:
   * Represents the standard deviations (uncertainty) for vision measurements.
   * First number: X position uncertainty in meters
   * Second number: Y position uncertainty in meters
   * Third number: rotation uncertainty in radians
   * 
   * Higher values = less trust in vision
   * Lower values = more trust in vision
   */
  public enum Cameras {
    /**
     * Center Camera
     * //
     */
    LEFT_CAM("Left",
        new Rotation3d(0, 0, Units.degreesToRadians(-30)),
        new Translation3d(Units.inchesToMeters(15),
            Units.inchesToMeters(11.75),
            Units.inchesToMeters(8)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getReefTagIDs()),
    RIGHT_CAM("Right",
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(30)),
        new Translation3d(Units.inchesToMeters(15),
            Units.inchesToMeters(-11.75),
            Units.inchesToMeters(8)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getReefTagIDs()),
    CENTER_CAM("Middle",
        new Rotation3d(Units.degreesToRadians(0
        ), Units.degreesToRadians(-43), Units.degreesToRadians(177)),
        new Translation3d(Units.inchesToMeters(8),
            Units.inchesToMeters(0),
            Units.inchesToMeters(41)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getHumanPlayerTagIDs());

        //Aarush Gota was here :0

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private static boolean updatedCache = false;

    private int[] allowedTagIDs;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated
     *                              poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated
     *                              poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix, int[] allowedTagIDs) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(PhotonVision.fieldLayout,
          PoseStrategy.AVERAGE_BEST_TARGETS,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
      this.allowedTagIDs = allowedTagIDs;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    public void clearCache() {
      resultsList.clear();
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = 0;
      amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      // System.err.println("Checkpoint 5");
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      // System.out.println("Results: " + resultsList.size());
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      // System.out.println("Cts: " + currentTimestamp + "Mrt: " + lastReadTimestamp +
      // " current - mostRecent " + (currentTimestamp - mostRecentTimestamp + "
      // debounce" + debounceTime));
      // if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >=
      // debounceTime)) &&
      // (currentTimestamp - lastReadTimestamp) >= debounceTime)
      // {
      if (true) {
        // System.err.println("Checkpoint 6");
        resultsList.clear();
        ;
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty()) {
          if (resultsList.get(0).targets.size() > 0) {
            updatedCache = true;
          }
          // System.err.println(resultsList.get(0).targets.size());
          // updatedCache = true;
          // System.err.println("Checkpoint 7");
          updateEstimatedGlobalPose();
        }
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      // for (var change : resultsList) {
      //   visionEst = poseEstimator.update(change);
      //   // try {
      //   // if (visionEst != null)
      //   // // System.out.println("Updated Pose " + visionEst.get().estimatedPose.getX()
      //   // + "y: " + visionEst.get().estimatedPose.getY());
      //   // } catch (Exception e) {
      //   // // TODO: handle exception
      //   // System.out.println("Pose died!?! ( big problem !!!)");
      //   // }
      //   updateEstimationStdDevs(visionEst, change.getTargets());
      // }
      // estimatedRobotPose = visionEst;

      // If you don't need target filtering, comment everything below this and uncomment the top part.

        for (var result : resultsList) {
            // Skip this result if there are no targets
            if (!result.hasTargets()) {
                continue;
            }
            
            // Check if this camera has tag filtering
            if (allowedTagIDs != null && allowedTagIDs.length > 0) {
                // Check if any of the targets match our allowed tag IDs
                boolean hasAllowedTag = false;
                for (PhotonTrackedTarget target : result.getTargets()) {
                    for (int id : allowedTagIDs) {
                        if (target.getFiducialId() == id) {
                            hasAllowedTag = true;
                            break;
                        }
                    }
                    if (hasAllowedTag) break;
                }
                
                // Skip this result if it doesn't have any allowed tags
                if (!hasAllowedTag) {
                    continue;
                }
            }
            
            // Update with the result
            visionEst = poseEstimator.update(result);
            
            // After getting the pose estimate, verify it used allowed tags if filtering is enabled
            if (visionEst.isPresent() && allowedTagIDs != null && allowedTagIDs.length > 0) {
                boolean usedAllowedTag = false;
                for (PhotonTrackedTarget usedTarget : visionEst.get().targetsUsed) {
                    for (int id : allowedTagIDs) {
                        if (usedTarget.getFiducialId() == id) {
                            usedAllowedTag = true;
                            break;
                        }
                    }
                    if (usedAllowedTag) break;
                }
                
                // If the pose didn't use any allowed tags, discard it
                if (!usedAllowedTag) {
                    visionEst = Optional.empty();
                    continue;
                }
            }
            
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }

    public void disableCamera() {
      // camera.setPipelineIndex(1);
      // camera.setDriverMode(true);
    }

    public void enableCamera() {
      // camera.setPipelineIndex(0);
      
      // camera.setDriverMode(false);

    }
  }

  /**
   * Gets a list of tag IDs that are on the human player station (both sides)
   * 
   * @return Array of human player station tag IDs
   */
  public static int[] getHumanPlayerTagIDs() {
    // In the 2025 Reefscape field, tags 1,2,12,13 are human player station tags
    // Modify these values based on the actual game field
    return new int[] { 1, 2, 12, 13};
  }

  /**
   * Gets a list of tag IDs that are on the Reefs (both sides)
   * @return
   */
  public static int[] getReefTagIDs(){
    // 2025 reefscape field - tags 6-11 on red side, 17-22 on blue side
    return new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  }

  public void disableFrontCameras(){
    humanPlayerFlag = true;
  }

  public void enableFrontCameras(){
    humanPlayerFlag = false;
  }
}

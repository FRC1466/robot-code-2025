// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  public static LoggedTunableNumber pitchTuning = new LoggedTunableNumber("Pitch Tuning", 30);
<<<<<<< Updated upstream
  public static final String kCameraNames[] = {
    "Camera_FrontLeft",
    "Camera_FrontRight",
=======
  public static final String kCameraNames[] = {"Camera_DevFront"
>>>>>>> Stashed changes
    // "Camera_BackLeft"
    "Camera_BackRight"
  };
  public static final Transform3d kRobotToCams[] = {
    new Transform3d(new Translation3d(.2921, -0.1651, .2), new Rotation3d(0, 0, 0))
    // new Transform3d(new Translation3d(.267, .292, .2), new Rotation3d(0, 0, 0)),
    // new Transform3d(new Translation3d(.267, -.292, .2), new Rotation3d(0, 0, 0)),
    /*   new Transform3d(
    new Translation3d(-.267, .278, .2), new Rotation3d(0, -Math.PI / 6, Math.PI * 3 / 4)),*/
    new Transform3d(
        new Translation3d(-.267, -.278, .2),
        new Rotation3d(0, Units.degreesToRadians(pitchTuning.getAsDouble()), Math.PI))
  };
  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(8, 8, 16);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(4, 4, 8);

  private PhotonCamera[] cameras;
  private PhotonPoseEstimator[] photonEstimators;
  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim[] cameraSims;
  private VisionSystemSim visionSim;

  public Vision() {
    cameras = new PhotonCamera[kCameraNames.length];
    photonEstimators = new PhotonPoseEstimator[kCameraNames.length];
    cameraSims = new PhotonCameraSim[kCameraNames.length];

    for (int i = 0; i < kCameraNames.length; i++) {
      cameras[i] = new PhotonCamera(kCameraNames[i]);
      photonEstimators[i] =
          new PhotonPoseEstimator(
              kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCams[i]);
      photonEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);

      // Create simulated camera for each physical camera
      for (int i = 0; i < cameras.length; i++) {
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.50, 0.20);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSims[i] = new PhotonCameraSim(cameras[i], cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSims[i], kRobotToCams[i]);

        cameraSims[i].enableDrawWireframe(true);
      }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    List<PhotonTrackedTarget> allDetectedTags = new ArrayList<>();

    // Try each camera until we get a valid result
    for (int i = 0; i < kCameraNames.length; i++) {
      var result = cameras[i].getLatestResult();
      if (result != null) {
        allDetectedTags.addAll(result.getTargets());

        // Log individual camera data - do this for each camera
        logCameraData(cameras[i], result.getTargets());

        var est = photonEstimators[i].update(result);
        if (est.isPresent()) {
          visionEst = est;
          updateEstimationStdDevs(Optional.of(est.get()), result.getTargets());
          break;
        }
      }
      if (visionEst.isPresent()) break;
    }

    // Log the vision pose estimation
    visionEst.ifPresent(
        est -> {
          /*    Logger.recordOutput("Vision/EstimatedPose", est.estimatedPose.toPose2d());
          Logger.recordOutput("Vision/EstimationTimestamp", est.timestampSeconds);*/

          // Create array of field poses for all detected tags
          Pose2d[] tagPoses = new Pose2d[est.targetsUsed.size()];
          int[] tagIDs = new int[est.targetsUsed.size()];

          for (int i = 0; i < est.targetsUsed.size(); i++) {
            PhotonTrackedTarget target = est.targetsUsed.get(i);
            int tagID = target.getFiducialId();
            tagIDs[i] = tagID;

            var optTagPose = kTagLayout.getTagPose(tagID);
            if (optTagPose.isPresent()) {
              tagPoses[i] = optTagPose.get().toPose2d();
            } else {
              tagPoses[i] = new Pose2d();
            }
          }

          /*  Logger.recordOutput("Vision/DetectedTagIDs", tagIDs);
          Logger.recordOutput("Vision/DetectedTagPoses", tagPoses);
          Logger.recordOutput("Vision/NumberOfDetectedTags", est.targetsUsed.size());*/
        });

    // Log if no estimation was found but tags were detected
    if (visionEst.isEmpty() && !allDetectedTags.isEmpty()) {
      // Log all detected tags even if no pose estimation was possible
      int[] tagIDs = new int[allDetectedTags.size()];
      Pose2d[] tagPoses = new Pose2d[allDetectedTags.size()];

      for (int i = 0; i < allDetectedTags.size(); i++) {
        PhotonTrackedTarget target = allDetectedTags.get(i);
        tagIDs[i] = target.getFiducialId();

        var optTagPose = kTagLayout.getTagPose(target.getFiducialId());
        if (optTagPose.isPresent()) {
          tagPoses[i] = optTagPose.get().toPose2d();
        } else {
          tagPoses[i] = new Pose2d();
        }
      }

      /*  Logger.recordOutput("Vision/DetectedTagIDs", tagIDs);
      Logger.recordOutput("Vision/DetectedTagPoses", tagPoses);
      Logger.recordOutput("Vision/NumberOfDetectedTags", allDetectedTags.size());
      Logger.recordOutput("Vision/EstimatedPose", new Pose2d());*/
    }

    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }

    // Call the logSeenAprilTags method to ensure it runs every cycle
    logSeenAprilTags();

    return visionEst;
  }

  /** Helper method to log data from individual cameras */
  private void logCameraData(PhotonCamera camera, List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) return;

    int[] tagIDs = new int[targets.size()];
    Pose3d[] tagPoses = new Pose3d[targets.size()];

    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);
      tagIDs[i] = target.getFiducialId();

      var optTagPose = kTagLayout.getTagPose(target.getFiducialId());
      if (optTagPose.isPresent()) {
        tagPoses[i] = optTagPose.get();
      } else {
        tagPoses[i] = new Pose3d();
      }
    }

    /*  Logger.recordOutput("Vision/" + camera.getName() + "/DetectedTagIDs", tagIDs);
    Logger.recordOutput("Vision/" + camera.getName() + "/DetectedTagPoses", tagPoses);
    Logger.recordOutput("Vision/" + camera.getName() + "/NumberOfDetectedTags", targets.size());*/
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimators[0].getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    if (Robot.isSimulation()) {
      visionSim.update(robotSimPose);
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      visionSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  /** Logs all AprilTags that have been seen at least once during the match */
  private final Set<Integer> allSeenTagIds = new HashSet<>();

  private final Set<Integer> currentlyVisibleTagIds = new HashSet<>();
  private long lastLogTime = 0;

  private void logSeenAprilTags() {
    // Clear current visible tags before repopulating
    currentlyVisibleTagIds.clear();
    boolean updatedSet = false;
    Map<Integer, Pose3d> seenTagsWithPoses = new HashMap<Integer, Pose3d>();

    // Check all cameras for detected tags
    for (PhotonCamera camera : cameras) {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          int id = target.getFiducialId();
          var tagPose = kTagLayout.getTagPose(id);

          if (tagPose.isPresent()) {
            // Record that this tag is currently visible
            currentlyVisibleTagIds.add(id);
            seenTagsWithPoses.put(id, tagPose.get());

            // Also add to historical set if new
            if (!allSeenTagIds.contains(id)) {
              allSeenTagIds.add(id);
              updatedSet = true;
            }
          }
        }
      }
    }

    // Only update the log if we've seen new tags or every second to maintain persistence
    long currentTime = System.currentTimeMillis();
    // Convert collections to arrays for logging - historical tags
    int[] allSeenIds = allSeenTagIds.stream().mapToInt(Integer::intValue).sorted().toArray();

    // Create arrays of currently visible tags
    int[] currentlyVisibleIds =
        currentlyVisibleTagIds.stream().mapToInt(Integer::intValue).sorted().toArray();

    // Create arrays of poses for visible tags
    Pose3d[] visibleTagPoses = new Pose3d[currentlyVisibleTagIds.size()];
    int index = 0;
    for (Integer id : currentlyVisibleIds) {
      visibleTagPoses[index++] = seenTagsWithPoses.get(id);
    }

    // Create arrays of poses for historical tags
    Pose3d[] allTagPoses = new Pose3d[allSeenTagIds.size()];
    index = 0;
    for (Integer id : allSeenIds) {
      // Get the pose from the map or layout if available
      if (seenTagsWithPoses.containsKey(id)) {
        allTagPoses[index] = seenTagsWithPoses.get(id);
      } else {
        var layoutPose = kTagLayout.getTagPose(id);
        allTagPoses[index] = layoutPose.orElse(new Pose3d());
      }
      index++;
    }

    // Log currently visible IDs and poses
    //  Logger.recordOutput("Vision/CurrentlyVisibleTagIDs", currentlyVisibleIds);
    //  Logger.recordOutput("Vision/CurrentlyVisibleTagPoses", visibleTagPoses);
    //   Logger.recordOutput("Vision/CurrentlyVisibleTagCount", currentlyVisibleIds.length);*/

    // Log all tags and poses that have ever been seen (every second)
    if (updatedSet || (currentTime - lastLogTime > 50)) {
      /*  Logger.recordOutput("Vision/AllSeenAprilTagIDs", allSeenIds);
      Logger.recordOutput("Vision/AllSeenAprilTagPoses", allTagPoses);
      Logger.recordOutput("Vision/TotalUniqueTagsSeen", allSeenIds.length);*/
      lastLogTime = currentTime;
    }

    lastLogTime = currentTime;
  }
}

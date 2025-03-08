// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  // Collections to manage multiple cameras
  private final Map<String, PhotonCamera> cameras = new HashMap<>();
  private final Map<String, PhotonPoseEstimator> photonEstimators = new HashMap<>();
  private final Map<String, PhotonCameraSim> cameraSims = new HashMap<>();

  // Current standard deviations of the vision pose estimator
  private Matrix<N3, N1> curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
  private VisionSystemSim visionSim;

  // List of all camera names to use in simulation
  private static final String[] SIM_CAMERA_NAMES = {
    "Camera_FrontLeft", "Camera_FrontRight", "Camera_BackLeft", "Camera_BackRight"
  };

  public Vision() {
    // Initialize each camera and its estimator
    for (String cameraName : VisionConstants.CAMERA_NAMES) {
      PhotonCamera camera = new PhotonCamera(cameraName);
      cameras.put(cameraName, camera);

      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(
              VisionConstants.TAG_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              VisionConstants.CAMERA_TRANSFORMS.get(cameraName));
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonEstimators.put(cameraName, estimator);
    }

    // Initialize simulation if needed
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

      // For simulation, we'll use all four cameras regardless of what's enabled on the real robot
      for (String cameraName : SIM_CAMERA_NAMES) {
        // Create the camera if it doesn't exist already
        if (!cameras.containsKey(cameraName)) {
          PhotonCamera camera = new PhotonCamera(cameraName);
          cameras.put(cameraName, camera);
        }

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
            VisionConstants.SIM_CAMERA_WIDTH_PX,
            VisionConstants.SIM_CAMERA_HEIGHT_PX,
            Rotation2d.fromDegrees(VisionConstants.SIM_CAMERA_FOV_DEG));
        cameraProp.setCalibError(
            VisionConstants.SIM_CAMERA_CALIB_ERROR_X, VisionConstants.SIM_CAMERA_CALIB_ERROR_Y);
        cameraProp.setFPS(VisionConstants.SIM_CAMERA_FPS);
        cameraProp.setAvgLatencyMs(VisionConstants.SIM_CAMERA_AVG_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionConstants.SIM_CAMERA_LATENCY_STD_DEV_MS);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cameras.get(cameraName), cameraProp);
        cameraSim.enableDrawWireframe(true);
        cameraSims.put(cameraName, cameraSim);

        visionSim.addCamera(cameraSim, VisionConstants.CAMERA_TRANSFORMS.get(cameraName));
      }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data, combining data from all cameras.
   * This may be empty. This should only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // Lists to collect all targets and estimates
    List<PhotonTrackedTarget> allTargets = new ArrayList<>();
    List<EstimatedRobotPose> allEstimates = new ArrayList<>();
    Map<String, Optional<EstimatedRobotPose>> cameraEstimates = new HashMap<>();

    // Process results from each camera
    for (String cameraName : VisionConstants.CAMERA_NAMES) {
      PhotonCamera camera = cameras.get(cameraName);
      PhotonPoseEstimator estimator = photonEstimators.get(cameraName);

      // Process all unread results from this camera
      for (var result : camera.getAllUnreadResults()) {
        if (result.hasTargets()) {
          Optional<EstimatedRobotPose> cameraEstimate = estimator.update(result);

          cameraEstimates.put(cameraName, cameraEstimate);

          if (cameraEstimate.isPresent()) {
            allEstimates.add(cameraEstimate.get());
            allTargets.addAll(result.getTargets());

            // Log individual camera estimates
            Logger.recordOutput(
                "Vision/" + cameraName, cameraEstimate.get().estimatedPose.toPose2d());
          } else {
            Logger.recordOutput("Vision/" + cameraName, new Pose2d());
          }
        }
      }
    }

    // Return empty if no valid estimates were found
    if (allEstimates.isEmpty()) {
      return Optional.empty();
    }

    // Calculate the combined pose by averaging all valid pose estimates
    double totalX = 0;
    double totalY = 0;
    double totalCos = 0;
    double totalSin = 0;
    double latestTimestamp = 0;

    for (EstimatedRobotPose estimate : allEstimates) {
      Pose2d pose = estimate.estimatedPose.toPose2d();
      totalX += pose.getX();
      totalY += pose.getY();

      // Add rotation components for proper angle averaging
      totalCos += Math.cos(pose.getRotation().getRadians());
      totalSin += Math.sin(pose.getRotation().getRadians());

      // Keep track of the latest timestamp
      latestTimestamp = Math.max(latestTimestamp, estimate.timestampSeconds);
    }

    // Calculate average pose
    double avgX = totalX / allEstimates.size();
    double avgY = totalY / allEstimates.size();
    double avgRotation = Math.atan2(totalSin, totalCos);

    Pose2d combinedPose = new Pose2d(avgX, avgY, new Rotation2d(avgRotation));

    // Create combined estimate with the latest timestamp
    EstimatedRobotPose combinedEstimate =
        new EstimatedRobotPose(
            new Pose3d(combinedPose),
            latestTimestamp,
            allTargets,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

    // Log the combined estimate
    Logger.recordOutput("Vision/CombinedEstimate", combinedPose);

    // Update standard deviations based on all targets
    updateEstimationStdDevs(Optional.of(combinedEstimate), allTargets);

    // Visualization for simulation
    if (Robot.isSimulation()) {
      getSimDebugField().getObject("VisionEstimation").setPose(combinedPose);

      // Also visualize individual camera estimates
      for (Map.Entry<String, Optional<EstimatedRobotPose>> entry : cameraEstimates.entrySet()) {
        entry
            .getValue()
            .ifPresent(
                est ->
                    getSimDebugField()
                        .getObject("Vision_" + entry.getKey())
                        .setPose(est.estimatedPose.toPose2d()));
      }
    }

    return Optional.of(combinedEstimate);
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
      curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
      return;
    }

    // Pose present. Start running Heuristic
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (var tgt : targets) {
      var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tgt.getFiducialId());
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
      curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    } else {
      // One or more tags visible, run the full heuristic.
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > VisionConstants.MAX_SINGLE_TAG_DISTANCE)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
      curStdDevs = estStdDevs;
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
      Logger.recordOutput("SimPose", robotSimPose);
      SmartDashboard.putData("Debug Field", getSimDebugField());
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  /** Logs all AprilTags currently seen by any camera */
  public void logSeenAprilTags() {
    Map<String, List<Pose3d>> cameraTagMap = new HashMap<>();
    List<Pose3d> allSeenTagPoses = new ArrayList<>();

    // Process each camera
    for (String cameraName : VisionConstants.CAMERA_NAMES) {
      PhotonCamera camera = cameras.get(cameraName);
      var result = camera.getLatestResult();
      List<Pose3d> cameraTagPoses = new ArrayList<>();

      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(target.getFiducialId());
          if (tagPose.isPresent()) {
            // Add to both the camera-specific and all tags lists
            cameraTagPoses.add(tagPose.get());
            allSeenTagPoses.add(tagPose.get());

            // Also log individual tag IDs and ambiguity for each camera
            Logger.recordOutput(
                "Vision/" + cameraName + "/Tag_" + target.getFiducialId() + "/ID",
                target.getFiducialId());
            Logger.recordOutput(
                "Vision/" + cameraName + "/Tag_" + target.getFiducialId() + "/Ambiguity",
                target.getPoseAmbiguity());
          }
        }
      }

      // Store tags seen by this camera
      cameraTagMap.put(cameraName, cameraTagPoses);

      // Log tags seen by this camera
      Logger.recordOutput(
          "Vision/" + cameraName + "/SeenTags", cameraTagPoses.toArray(new Pose3d[0]));

      // Log the number of tags seen by this camera
      Logger.recordOutput("Vision/" + cameraName + "/TagCount", cameraTagPoses.size());
    }

    // Log the array of Pose3d objects from all cameras
    Logger.recordOutput("Vision/AllSeenTags", allSeenTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/TotalTagCount", allSeenTagPoses.size());
  }
}

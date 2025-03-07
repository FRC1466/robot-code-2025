// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  private final PhotonCamera cameraFL;
  private final PhotonCamera cameraFR;
  private final PhotonCamera cameraBL;
  private final PhotonCamera cameraBR;
  private final PhotonPoseEstimator photonEstimatorFL;
  private final PhotonPoseEstimator photonEstimatorFR;
  private final PhotonPoseEstimator photonEstimatorBL;
  private final PhotonPoseEstimator photonEstimatorBR;
  private Matrix<N3, N1> curStdDevs;

  // For running the cameras on separate threads
  private final Notifier allNotifier;

  // Simulation
  private PhotonCameraSim cameraSimFL;
  private PhotonCameraSim cameraSimFR;
  private PhotonCameraSim cameraSimBL;
  private PhotonCameraSim cameraSimBR;

  private VisionSystemSim visionSim;

  // Latest results from cameras
  private EstimatedRobotPose latestEstimateFL = null;
  private EstimatedRobotPose latestEstimateFR = null;
  private EstimatedRobotPose latestEstimateBL = null;
  private EstimatedRobotPose latestEstimateBR = null;

  // Timestamp for the last measurement
  private double lastMeasurementTimestamp = 0.0;

  public Vision() {
    Logger.recordOutput("Vision/Status", "Initializing");

    cameraFL = new PhotonCamera(VisionConstants.CAMERA_NAMES[0]);
    cameraFR = new PhotonCamera(VisionConstants.CAMERA_NAMES[1]);
    cameraBL = new PhotonCamera(VisionConstants.CAMERA_NAMES[2]);
    cameraBR = new PhotonCamera(VisionConstants.CAMERA_NAMES[3]);

    Logger.recordOutput(
        "Vision/Cameras",
        VisionConstants.CAMERA_NAMES[0]
            + ", "
            + VisionConstants.CAMERA_NAMES[1]
            + ", "
            + VisionConstants.CAMERA_NAMES[2]
            + ", "
            + VisionConstants.CAMERA_NAMES[3]);

    photonEstimatorFL =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[0]));
    photonEstimatorFL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorFR =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[1]));
    photonEstimatorFR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorBL =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[2]));
    photonEstimatorBL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorBR =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[3]));
    photonEstimatorBR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Create notifier to run camera updates on separate threads
    allNotifier = new Notifier(this::updateAllCameras);
    allNotifier.setName("VisionRunAll");
    allNotifier.startPeriodic(0.02); // 50Hz update rate
    Logger.recordOutput("Vision/Status", "Initialized with 50Hz camera thread");

    // ----- Simulation
    if (Robot.isSimulation()) {
      Logger.recordOutput("Vision/Mode", "Simulation");

      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(30);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSimFL = new PhotonCameraSim(cameraFL, cameraProp);
      cameraSimFR = new PhotonCameraSim(cameraFR, cameraProp);
      cameraSimBL = new PhotonCameraSim(cameraBL, cameraProp);
      cameraSimBR = new PhotonCameraSim(cameraBR, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(
          cameraSimFL, VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[0]));
      visionSim.addCamera(
          cameraSimFR, VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[1]));
      visionSim.addCamera(
          cameraSimBL, VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[2]));
      visionSim.addCamera(
          cameraSimBR, VisionConstants.CAMERA_TRANSFORMS.get(VisionConstants.CAMERA_NAMES[3]));

      cameraSimFL.enableDrawWireframe(true);
      cameraSimFR.enableDrawWireframe(true);
      cameraSimBL.enableDrawWireframe(true);
      cameraSimBR.enableDrawWireframe(true);
    } else {
      Logger.recordOutput("Vision/Mode", "Real");
    }
  }

  /** Updates all camera estimates in a single method - to be called by the notifier */
  private void updateAllCameras() {
    updateCameraEstimate(cameraFL, photonEstimatorFL, result -> latestEstimateFL = result, "FL");
    updateCameraEstimate(cameraFR, photonEstimatorFR, result -> latestEstimateFR = result, "FR");
    updateCameraEstimate(cameraBL, photonEstimatorBL, result -> latestEstimateBL = result, "BL");
    updateCameraEstimate(cameraBR, photonEstimatorBR, result -> latestEstimateBR = result, "BR");
  }

  /** Updates a single camera's estimate and sets the result using the provided callback */
  private void updateCameraEstimate(
      PhotonCamera camera,
      PhotonPoseEstimator estimator,
      java.util.function.Consumer<EstimatedRobotPose> resultConsumer,
      String cameraName) {

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      Logger.recordOutput("Vision/" + cameraName + "/ResultsCount", results.size());
    }

    for (PhotonPipelineResult result : results) {
      int numTargets = result.getTargets().size();

      if (numTargets > 0) {
        // Record detection info
        Logger.recordOutput("Vision/" + cameraName + "/TargetCount", numTargets);

        // Log target IDs for each camera
        int[] tagIds = new int[numTargets];
        for (int i = 0; i < numTargets; i++) {
          tagIds[i] = result.getTargets().get(i).getFiducialId();
        }
        Logger.recordOutput("Vision/" + cameraName + "/DetectedIDs", tagIds);
      }

      estimator
          .update(result)
          .ifPresent(
              estimated -> {
                updateEstimationStdDevs(Optional.of(estimated), result.getTargets());
                resultConsumer.accept(estimated);

                // Record the pose estimate
                Pose2d estimatedPose = estimated.estimatedPose.toPose2d();
                Logger.recordOutput("Vision/" + cameraName + "/EstimatedPose", estimatedPose);
                Logger.recordOutput(
                    "Vision/" + cameraName + "/TimestampSecs", estimated.timestampSeconds);

                // Record number of tags used for the estimate
                Logger.recordOutput(
                    "Vision/" + cameraName + "/TagsUsedCount", estimated.targetsUsed.size());
              });
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. Returns the
   * best estimate from any camera that has a valid estimate.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // Return the most recent valid estimate from any camera
    String cameraUsed = "None";
    EstimatedRobotPose result = null;

    if (latestEstimateFL != null) {
      cameraUsed = "FL";
      result = latestEstimateFL;
    } else if (latestEstimateFR != null) {
      cameraUsed = "FR";
      result = latestEstimateFR;
    } else if (latestEstimateBL != null) {
      cameraUsed = "BL";
      result = latestEstimateBL;
    } else if (latestEstimateBR != null) {
      cameraUsed = "BR";
      result = latestEstimateBR;
    }

    Logger.recordOutput("Vision/CameraUsedForPose", cameraUsed);

    if (result != null) {
      return Optional.of(result);
    }
    return Optional.empty();
  }

  /**
   * Process the latest vision measurement for use with a pose estimator
   *
   * @param poseEstimator The SwerveDrivePoseEstimator to update
   */
  public void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator) {
    // Count cameras with valid estimates
    int validEstimateCount = 0;
    if (latestEstimateFL != null) validEstimateCount++;
    if (latestEstimateFR != null) validEstimateCount++;
    if (latestEstimateBL != null) validEstimateCount++;
    if (latestEstimateBR != null) validEstimateCount++;

    Logger.recordOutput("Vision/ValidEstimateCount", validEstimateCount);

    // Process estimates from all cameras
    processCameraEstimate(latestEstimateFL, poseEstimator, "FL");
    processCameraEstimate(latestEstimateFR, poseEstimator, "FR");
    processCameraEstimate(latestEstimateBL, poseEstimator, "BL");
    processCameraEstimate(latestEstimateBR, poseEstimator, "BR");

    // Reset latest estimates after processing
    latestEstimateFL = null;
    latestEstimateFR = null;
    latestEstimateBL = null;
    latestEstimateBR = null;
  }

  /** Process a single camera estimate and add it to the pose estimator if valid */
  private void processCameraEstimate(
      EstimatedRobotPose estimate, SwerveDrivePoseEstimator poseEstimator, String cameraName) {
    if (estimate == null) return;

    var pose2d = estimate.estimatedPose.toPose2d();

    // Get current pose and calculate jump distance
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    double jumpDistance = currentPose.getTranslation().getDistance(pose2d.getTranslation());

    // Calculate time since last measurement
    double timeSinceLastMeasurement = Timer.getFPGATimestamp() - lastMeasurementTimestamp;
    double maxReasonableDistance =
        Constants.MAX_ROBOT_SPEED * timeSinceLastMeasurement + 0.1; // meters

    if (jumpDistance > maxReasonableDistance) {
      Logger.recordOutput("Vision/" + cameraName + "/RejectedJump", jumpDistance);
      return; // Reject this measurement
    }

    // Continue with existing confidence calculation and pose update
    Matrix<N3, N1> confidence = confidenceCalculator(estimate);

    // Log the actual values in the matrix
    Logger.recordOutput("Vision/" + cameraName + "/ConfidenceX", confidence.get(0, 0));
    Logger.recordOutput("Vision/" + cameraName + "/ConfidenceY", confidence.get(1, 0));
    Logger.recordOutput("Vision/" + cameraName + "/ConfidenceRot", confidence.get(2, 0));

    // Log the pose being added
    Logger.recordOutput("Vision/" + cameraName + "/MeasurementPose", pose2d);

    // Add the vision measurement to the pose estimator
    poseEstimator.addVisionMeasurement(pose2d, estimate.timestampSeconds, confidence);

    lastMeasurementTimestamp = Timer.getFPGATimestamp();
  }

  /** Calculates the confidence in a vision measurement based on distance and ambiguity */
  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance =
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance) smallestDistance = distance;
    }

    double poseAmbiguityFactor =
        estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(1, (estimation.targetsUsed.get(0).getPoseAmbiguity() + 0.05) * 5);

    double confidenceMultiplier =
        Math.max(
            1,
            (Math.max(1, Math.max(0, smallestDistance - 2.0) * 0.8) * poseAmbiguityFactor)
                / (1 + ((estimation.targetsUsed.size() - 1) * 0.5)));

    Logger.recordOutput("Vision/ConfidenceCalculation/SmallestDistance", smallestDistance);
    Logger.recordOutput("Vision/ConfidenceCalculation/TargetsUsed", estimation.targetsUsed.size());
    Logger.recordOutput("Vision/ConfidenceCalculation/AmbiguityFactor", poseAmbiguityFactor);
    Logger.recordOutput("Vision/ConfidenceCalculation/ConfidenceMultiplier", confidenceMultiplier);

    return VisionConstants.MULTI_TAG_STD_DEVS.times(confidenceMultiplier);
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
      Logger.recordOutput("Vision/StdDevs/Type", "Single Tag Default");
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimatorFL.getFieldTags().getTagPose(tgt.getFiducialId());
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
        Logger.recordOutput("Vision/StdDevs/Type", "No Tags Visible - Single Tag Default");
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        Logger.recordOutput("Vision/StdDevs/NumTags", numTags);
        Logger.recordOutput("Vision/StdDevs/AvgDistance", avgDist);

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
          Logger.recordOutput("Vision/StdDevs/Type", "Multiple Tags");
        } else {
          Logger.recordOutput("Vision/StdDevs/Type", "Single Tag");
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          Logger.recordOutput("Vision/StdDevs/Type", "Far Single Tag - Max Uncertainty");
        } else {
          double distanceFactor = 1 + (avgDist * avgDist / 30);
          Logger.recordOutput("Vision/StdDevs/DistanceFactor", distanceFactor);
          estStdDevs = estStdDevs.times(distanceFactor);
        }
        curStdDevs = estStdDevs;
      }
    }

    // Log the final standard deviation values
    Logger.recordOutput("Vision/StdDevs/X", curStdDevs.get(0, 0));
    Logger.recordOutput("Vision/StdDevs/Y", curStdDevs.get(1, 0));
    Logger.recordOutput("Vision/StdDevs/Rot", curStdDevs.get(2, 0));
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
    visionSim.update(robotSimPose);
    Logger.recordOutput("Vision/Simulation/RobotPose", robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      visionSim.resetRobotPose(pose);
      Logger.recordOutput("Vision/Simulation/ResetPose", pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}

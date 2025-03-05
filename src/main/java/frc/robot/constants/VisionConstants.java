// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Map;

public final class VisionConstants {

  // Camera names
  public static final String[] CAMERA_NAMES = {
    "Camera_FrontLeft", "Camera_FrontRight", "Camera_BackLeft", "Camera_BackRight"
  };

  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // Robot-to-camera transforms for each camera
  // These define the position and orientation of each camera relative to the robot center
  public static final Map<String, Transform3d> CAMERA_TRANSFORMS =
      Map.of(
          "Camera_FrontLeft",
          new Transform3d(new Translation3d(.267, .292, .2), new Rotation3d(0, 0, 0)),
          "Camera_FrontRight",
          new Transform3d(new Translation3d(.267, -.292, .2), new Rotation3d(0, 0, 0)),
          "Camera_BackLeft",
          new Transform3d(new Translation3d(-.267, .292, .2), new Rotation3d(0, 0, Math.PI)),
          "Camera_BackRight",
          new Transform3d(new Translation3d(-.267, -.292, .2), new Rotation3d(0, 0, Math.PI)));

  // Standard deviations for vision measurements (x, y, theta)
  // Smaller values = more trust in vision measurements

  // When using a single April tag for pose estimation
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(.1, .2, .3);

  // When using multiple April tags for pose estimation (more accurate)
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(.1, .1, .2);

  // Minimum ambiguity to accept a tag detection
  public static final double MAX_AMBIGUITY = 0.2;

  // Maximum distance to trust a single tag (in meters)
  public static final double MAX_SINGLE_TAG_DISTANCE = 4.0;

  // Camera properties for simulation
  public static final int SIM_CAMERA_WIDTH_PX = 960;
  public static final int SIM_CAMERA_HEIGHT_PX = 720;
  public static final double SIM_CAMERA_FOV_DEG = 90.0;
  public static final double SIM_CAMERA_FPS = 30.0;
  public static final double SIM_CAMERA_AVG_LATENCY_MS = 35.0; // Reduced from 50.0
  public static final double SIM_CAMERA_LATENCY_STD_DEV_MS = 5.0; // Reduced from 15.0
  public static final double SIM_CAMERA_CALIB_ERROR_X = 0.05; // Reduced from 0.35
  public static final double SIM_CAMERA_CALIB_ERROR_Y = 0.02; // Reduced from 0.10
}

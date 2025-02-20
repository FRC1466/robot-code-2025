// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import webblib.Gains;

public final class Constants {
  public static final Translation3d cameraTranslation = new Translation3d(0.28, 0.0, 0.23);
  public static final Rotation3d cameraRotation = new Rotation3d(0, Math.toRadians(-33.5), 0);

  public static final Mode simMode = Mode.REPLAY;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static void setRobot(RobotType type) {
    robotType = type;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  public static final class PoseEstimator {
    /** THANK YOU IRON PANTHERS */
    public static final double NOISY_DISTANCE_METERS = 2.5;

    /**
     * The number to multiply by the smallest of the distance minus the above constant, clamped
     * above 1 to be the numerator of the fraction.
     */
    public static final double DISTANCE_WEIGHT = 7;

    /**
     * The number to multiply by the number of tags beyond the first to get the denominator of the
     * deviations matrix.
     */
    public static final double TAG_PRESENCE_WEIGHT = 10;

    /** The amount to shift the pose ambiguity by before multiplying it. */
    public static final double POSE_AMBIGUITY_SHIFTER = .2;

    /** The amount to multiply the pose ambiguity by if there is only one tag. */
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  }

  public static final class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.1, .2, .3);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.1, .1, .2);
  }

  public static final class RotationConstants {
    public static final int armPort = 14, dutyCyclePort = 0;
    // DO NOT TOUCH!!!!
    public static final Gains rotationPosition = new Gains(.16, 0.13, 0.05, 0.0, 0, .2);
    public static final double restRadians = .05,
        coralPosRadians = .505,
        l4coralPosRadians = 1,
        algaePosition = 3;

    public static final double maxRadians = Math.PI;
    public static final double gravityFF = 0.014;
    public static final double absolutePositionOffset = -0.3211;
    public static final boolean encoderInverted = true;
    public static final double dutyCycleResolution = 1.0;

    public static final class RotationConfig {
      public static final CurrentLimitsConfigs supplyCurrent;

      public static final TalonFXConfiguration motorConfig;

      static {
        supplyCurrent = new CurrentLimitsConfigs();

        motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits = supplyCurrent;
      }
    }
  }

  public static class Vision {
    public static final String kCameraName = "Global_Shutter_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.3, -0.13, 0.22), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class ElevatorConstants {
    public static final int masterID = 17;
    public static final int slaveID = 16;

    public static final Gains elevatorPosition = new Gains(.15, 0.00, 0.00, .0, 0, 2.5);
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import webblib.Gains;

public final class Constants {
  public static boolean disableHAL = false;
  public static final boolean CHECK_ROBOT_TYPE = false; // Enables robot type check during builds

  public static final boolean tuningMode = true; // Allows changing of LoggedTunableNumbers
  public static final boolean PlayMusic = false;

  private static RobotType robotType = RobotType.SIMBOT;
  public static Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

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

  public static void setMode(Mode mode) {
    currentMode = mode;
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

  public static final class RotationConstants {
    public static final int armPort = 14, dutyCyclePort = 0;
    // DO NOT TOUCH!!!!
    public static final Gains rotationPosition = new Gains(.18, 0.02, 0.02, 0.0, 0, .25);
    public static final double restRadians = .0,
        coralPosRadians = .505,
        l4coralPosRadians = 1.15,
        algaePosition = Math.PI;

    public static final double maxRadians = Math.PI;
    public static final double gravityFF = 0.014;
    public static final double absolutePositionOffset = -0.4601;
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

  public static class ElevatorConstants {
    public static final int masterID = 17;
    public static final int slaveID = 16;

    public static final Gains elevatorPosition = new Gains(.15, 0.00, 0.004, .0, 0, .4);
  }
}

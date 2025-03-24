// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Telemetry {
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
  }

  /* LoggedMechanisms to represent the swerve module states */
  private final LoggedMechanism2d[] m_moduleLoggedMechanisms =
      new LoggedMechanism2d[] {
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final LoggedMechanismLigament2d[] m_moduleSpeeds =
      new LoggedMechanismLigament2d[] {
        m_moduleLoggedMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleLoggedMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleLoggedMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleLoggedMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final LoggedMechanismLigament2d[] m_moduleDirections =
      new LoggedMechanismLigament2d[] {
        m_moduleLoggedMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(
                new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleLoggedMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(
                new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleLoggedMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(
                new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleLoggedMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(
                new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /** Accept the swerve drive state and telemeterize it to Logger. */
  public void telemeterize(SwerveDriveState state) {
    /* Record the pose */
    Logger.recordOutput(
        "DriveState/CameraPose",
        new Pose3d(
            new Translation3d(state.Pose.getX() - .267, state.Pose.getY() + .278, .2),
            new Rotation3d(
                0, -Math.PI / 6, state.Pose.getRotation().getRadians() + Math.PI * 3 / 4)));

    /* Record the speeds */
    Logger.recordOutput("DriveState/Speeds", state.Speeds);

    /* Record module states, targets and positions */
    Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
    Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
    Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);

    /* Record timing information */
    Logger.recordOutput("DriveState/Timestamp", state.Timestamp);
    Logger.recordOutput("DriveState/OdometryPeriod", state.OdometryPeriod);
    Logger.recordOutput("DriveState/OdometryFrequency", 1.0 / state.OdometryPeriod);

    /* Also log arrays for compatibility with existing code */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    Logger.recordOutput("DriveState/PoseArray", m_poseArray);

    for (int i = 0; i < 4; ++i) {
      m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }
    Logger.recordOutput("DriveState/ModuleStatesArray", m_moduleStatesArray);
    Logger.recordOutput("DriveState/ModuleTargetsArray", m_moduleTargetsArray);

    /* Record field visualization data */
    Logger.recordOutput("Field/Robot", state.Pose);

    /* Update the module LoggedMechanisms and log them */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

      Logger.recordOutput("Module/" + i, m_moduleLoggedMechanisms[i]);
    }
  }
}

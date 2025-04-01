// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;

public class PathfindConstants {
  public static final Pose2d[][] blueTargetPoseReef = createReefPoses();

  @AutoLogOutput
  private static Pose2d[][] createReefPoses() {
    Pose2d[][] reefPoses = new Pose2d[6][2]; // 6 faces, left and right branches

    for (int i = 0; i < 6; i++) {
      Pose2d centerFace = FieldConstants.Reef.centerFaces[i];
      double approachDistance = 0.5; // Distance from the reef face
      double offsetY = 0.3; // Offset for left/right branches

      reefPoses[i][0] =
          new Pose2d(
              centerFace.getX()
                  - (approachDistance * Math.cos(centerFace.getRotation().getRadians())),
              centerFace.getY()
                  - (approachDistance * Math.sin(centerFace.getRotation().getRadians()))
                  + offsetY,
              centerFace.getRotation());

      reefPoses[i][1] =
          new Pose2d(
              centerFace.getX()
                  - (approachDistance * Math.cos(centerFace.getRotation().getRadians())),
              centerFace.getY()
                  - (approachDistance * Math.sin(centerFace.getRotation().getRadians()))
                  - offsetY,
              centerFace.getRotation());
    }

    return reefPoses;
  }

  // Red Target Poses for Station
  public static final Pose2d[] redTargetPoseStation = {
    new Pose2d(16.195, 7.19, Rotation2d.fromDegrees(-125.000)),
    new Pose2d(16.288, 0.937, Rotation2d.fromDegrees(125.000)),
  };

  public static final Pose2d redTargetPoseProcessor =
      new Pose2d(11.496, 7.495, Rotation2d.fromDegrees(90.000));

  public static final double redTargetPoseXBarge = 8.18;
}

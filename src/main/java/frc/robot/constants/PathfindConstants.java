// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathfindConstants {
  // TODO: the pose for index 2 is the same for both
  // Red Target Poses for Reef
  // 6-11 as 0-5 , First is Left and Second is Right
  public static final Pose2d[][] redTargetPoseReef = {
    {
      new Pose2d(13.550, 2.837, Rotation2d.fromDegrees(120)),
      new Pose2d(13.853, 3.003, Rotation2d.fromDegrees(120))
    },
    {
      new Pose2d(14.347, 3.860, Rotation2d.fromDegrees(-180)),
      new Pose2d(14.347, 4.190, Rotation2d.fromDegrees(-180))
    },
    {
      new Pose2d(13.52, 5.15, Rotation2d.fromDegrees(-120)),
      new Pose2d(13.49, 5.23, Rotation2d.fromDegrees(-120))
    },
    {
      new Pose2d(12.6, 5.19, Rotation2d.fromDegrees(-60)),
      new Pose2d(12.28, 5.04, Rotation2d.fromDegrees(-60))
    },
    {
      new Pose2d(11.695, 4.169, Rotation2d.fromDegrees(0)),
      new Pose2d(11.7, 3.83, Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(12.283, 2.998, Rotation2d.fromDegrees(60)),
      new Pose2d(12.554, 2.850, Rotation2d.fromDegrees(60))
    }
  };

  // Red Target Poses for Station
  public static final Pose2d[] redTargetPoseStation = {
    new Pose2d(16.195, 7.19, Rotation2d.fromDegrees(-125.000)),
    new Pose2d(16.288, 0.937, Rotation2d.fromDegrees(125.000)),
  };

  public static final Pose2d redTargetPoseProcessor =
      new Pose2d(11.496, 7.495, Rotation2d.fromDegrees(90.000));

  public static final double redTargetPoseXBarge = 10.3;
}

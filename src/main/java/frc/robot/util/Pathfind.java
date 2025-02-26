// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.text.ParseException;

public class Pathfind {
  PathPlannerPath path;
  PathConstraints constraints;
  static Command redPathfindingCommand;
  static Command bluePathfindingCommand;

  // String bestPath = "Tag 10 to G";
  // obj 0 is left, obj 1 is right
  // 6,7,8,9,10,11
  public Pose2d[][] redTargetPose = {
    {
      new Pose2d(13.550, 2.837, Rotation2d.fromDegrees(120)),
      new Pose2d(13.853, 3.003, Rotation2d.fromDegrees(120))
    },
    {
      new Pose2d(14.347, 3.860, Rotation2d.fromDegrees(-180)),
      new Pose2d(14.347, 4.190, Rotation2d.fromDegrees(-180))
    },
    {
      new Pose2d(13.840, 5.044, Rotation2d.fromDegrees(-120)),
      new Pose2d(13.560, 5.216, Rotation2d.fromDegrees(-120))
    },
    {
      new Pose2d(12.561, 5.237, Rotation2d.fromDegrees(-60)),
      new Pose2d(12.301, 12.301, Rotation2d.fromDegrees(-60))
    },
    {
      new Pose2d(11.695, 4.169, Rotation2d.fromDegrees(0)),
      new Pose2d(11.695, 3.860, Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(12.283, 2.998, Rotation2d.fromDegrees(60)),
      new Pose2d(12.554, 2.850, Rotation2d.fromDegrees(60))
    }
  };

  // DO NOT USE - NEEDS TO BE FIXED
  public Pose2d[][] blueTargetPoseTransformed = {
    {
      // 13.550 - 8.774 = 4.776; 300° becomes -60°
      new Pose2d(4.776, 2.837, Rotation2d.fromDegrees(-60)),
      // 13.853 - 8.774 = 5.079; 300° becomes -60°
      new Pose2d(5.079, 3.003, Rotation2d.fromDegrees(-60))
    },
    {
      // 14.347 - 8.774 = 5.573; 0° remains 0°
      new Pose2d(5.573, 3.860, Rotation2d.fromDegrees(0)),
      // 14.347 - 8.774 = 5.573; 0° remains 0°
      new Pose2d(5.573, 4.190, Rotation2d.fromDegrees(0))
    },
    {
      // 13.840 - 8.774 = 5.066; 60° remains 60°
      new Pose2d(5.066, 5.044, Rotation2d.fromDegrees(60)),
      // 13.560 - 8.774 = 4.786; 60° remains 60°
      new Pose2d(4.786, 5.216, Rotation2d.fromDegrees(60))
    },
    {
      // 12.561 - 8.774 = 3.787; 120° remains 120°
      new Pose2d(3.787, 5.237, Rotation2d.fromDegrees(120)),
      // 12.301 - 8.774 = 3.527; 120° remains 120°
      new Pose2d(3.527, 12.301, Rotation2d.fromDegrees(120))
    },
    {
      // 11.695 - 8.774 = 2.921; 180° remains 180° (unchanged as it's not greater than 180)
      new Pose2d(2.921, 4.169, Rotation2d.fromDegrees(180)),
      // 11.695 - 8.774 = 2.921; 180° remains 180°
      new Pose2d(2.921, 3.860, Rotation2d.fromDegrees(180))
    },
    {
      // 12.283 - 8.774 = 3.509; 240° becomes -120° (240 - 360)
      new Pose2d(3.509, 2.998, Rotation2d.fromDegrees(-120)),
      // 12.554 - 8.774 = 3.780; 240° becomes -120°
      new Pose2d(3.780, 2.850, Rotation2d.fromDegrees(-120))
    }
  };

  public Pose2d targetPose = new Pose2d(11.638, 3.863, Rotation2d.fromDegrees(0));

  public Pathfind() throws IOException, ParseException {
    /*  Load the path we want to pathfind to and follow
    try {
      path = PathPlannerPath.fromPathFile(bestPath);
    } catch (FileVersionException | IOException | org.json.simple.parser.ParseException e) {
      System.err.println("Error loading path: " + e.getMessage());
      e.printStackTrace();
    }*/

    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path.
    constraints =
        new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

  }

  public Command getPathfindingCommand(int closestTag, int leftOrRight) {

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            redTargetPose[closestTag][leftOrRight],
            constraints,
            0.0 // Goal end velocity in meters/sec
            );
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            blueTargetPoseTransformed[closestTag][leftOrRight],
            constraints,
            0.0 // Goal end velocity in meters/sec
            );
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return redPathfindingCommand;
    }

    return bluePathfindingCommand;
  }
}

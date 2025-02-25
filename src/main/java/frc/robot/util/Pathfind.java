// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.text.ParseException;

public class Pathfind {
  PathPlannerPath path;
  PathConstraints constraints;
  static Command pathfindingCommand;
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

    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.0 // Goal end velocity in meters/sec
            );
  }

  public Command getPathfindingCommand() {
    return pathfindingCommand;
  }
}

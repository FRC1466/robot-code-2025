// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.onTheFlyPaths;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import java.util.List;

public class TestPath {
  public static PathPlannerPath createTestPath(CommandSwerveDrivetrain drivetrain) {
    try {
      // Create a list of waypoints from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic
      // rotation.
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              drivetrain.getState().Pose, new Pose2d(5.910, 3.880, Rotation2d.fromDegrees(0)));

      PathConstraints constraints =
          new PathConstraints(.5, .5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

      // Create the path using the waypoints created above
      PathPlannerPath TestPath =
          new PathPlannerPath(
              waypoints,
              constraints,
              null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be
              // null for on-the-fly paths.
              new GoalEndState(
                  0.0,
                  Rotation2d.fromDegrees(
                      0)) // Goal end state. You can set a holonomic rotation here. If using a
              // differential drivetrain, the rotation will have no effect.
              );

      // Prevent the path from being flipped if the coordinates are already correct
      TestPath.preventFlipping = true;

      return TestPath;
    } catch (Exception e) {
      DriverStation.reportError(
          "Error creating PathPlannerPath: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public static Command getPathCommand(CommandSwerveDrivetrain drivetrain) {
    try {
      PathPlannerPath path = createTestPath(drivetrain);
      if (path == null) {
        throw new RuntimeException("PathPlannerPath creation failed.");
      }
      return drivetrain.followPathCommand(path);
    } catch (Exception e) {
      DriverStation.reportError(
          "Could not instantiate PathPlannerPath: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }
}

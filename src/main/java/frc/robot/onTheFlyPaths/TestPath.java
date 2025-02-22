// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.onTheFlyPaths;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import java.util.List;

public class TestPath {
  public static PathPlannerPath createTestPath() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(new Pose2d(5.910, 3.880, Rotation2d.fromDegrees(0)));

    PathConstraints constraints =
        new PathConstraints(.5, .5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

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
    // path.preventFlipping = true;

    return TestPath;
  }

  public static Command getPathCommand(CommandSwerveDrivetrain drivetrain) {
    PathPlannerPath path = createTestPath();
    return drivetrain.followPathCommand(path);
  }
}

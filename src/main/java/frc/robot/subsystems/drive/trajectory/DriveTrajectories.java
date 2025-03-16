// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;
import java.util.function.Function;
import lombok.experimental.ExtensionMethod;

/**
 * Contains predefined trajectories for the robot to follow. Teams should define their game-specific
 * trajectories in this class following the examples provided.
 */
@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();

  // List of functions that take a set of completed paths and return a map of
  // trajectories to generate (or null if they cannot be generated yet)
  public static final List<Function<Set<String>, Map<String, List<PathSegment>>>> suppliedPaths =
      new ArrayList<>();

  // Example paths - used for preload of trajectory classes in drive constructor
  static {
    // A simple straight line path
    paths.put(
        "driveStraight",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(new Pose2d(10, 4, Rotation2d.fromDegrees(0)))
                .addPoseWaypoint(new Pose2d(11.5, 4, Rotation2d.fromDegrees(0)))
                .build()));

    // A more complex path with multiple waypoints
    paths.put(
        "examplePath",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(Pose2d.kZero)
                .addTranslationWaypoint(new Translation2d(2, 3))
                .addPoseWaypoint(new Pose2d(0, 3, Rotation2d.fromDegrees(270.0)))
                .addPoseWaypoint(new Pose2d(2, 0.6, Rotation2d.fromDegrees(30.0)))
                .build()));
  }

  /** Returns the last waypoint of a trajectory. */
  public static Waypoint getLastWaypoint(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    return trajectory
        .get(trajectory.size() - 1)
        .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.trajectory.HolonomicTrajectory;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Utility class containing helper methods for autonomous routines.
 *
 * <p>This class provides static methods for common autonomous operations such as:
 *
 * <ul>
 *   <li>Resetting robot pose
 *   <li>Following trajectories
 *   <li>Position verification
 *   <li>Coordinate transformations for alliance-specific behavior
 * </ul>
 */
public class AutoCommands {
  /** Private constructor to prevent instantiation of utility class. */
  private AutoCommands() {}

  /**
   * Creates a command that resets the robot's odometry to match a trajectory's starting pose.
   *
   * @param trajectory The trajectory to use for pose reset
   * @param mirror Whether to mirror the pose across the field (for symmetrical field elements)
   * @return A command that resets the robot pose
   */
  public static Command resetPoseCommand(
      CommandSwerveDrivetrain drive, HolonomicTrajectory trajectory, boolean mirror) {
    return Commands.runOnce(() -> resetPose(drive, trajectory, mirror));
  }

  /**
   * Resets the robot's odometry to match a trajectory's starting pose.
   *
   * @param trajectory The trajectory to use for pose reset
   * @param mirror Whether to mirror the pose across the field (for symmetrical field elements)
   */
  public static void resetPose(
      CommandSwerveDrivetrain drive, HolonomicTrajectory trajectory, boolean mirror) {
    drive.resetPose(
        AllianceFlipUtil.apply(
            mirror ? mirrorPose(trajectory.getStartPose()) : trajectory.getStartPose()));
  }

  /**
   * Creates a command that resets the robot's odometry to a specific pose.
   *
   * @param pose The pose to reset to
   * @param applyAllianceFlip Whether to flip the pose based on alliance color
   * @return A command that resets the robot pose
   */
  public static Command resetPoseCommand(
      CommandSwerveDrivetrain drive, Pose2d pose, boolean applyAllianceFlip) {
    return Commands.runOnce(
        () -> drive.resetPose(applyAllianceFlip ? AllianceFlipUtil.apply(pose) : pose));
  }

  /**
   * Creates a trajectory following command with an optional target pose.
   *
   * @param drive The drive subsystem
   * @param trajectory The trajectory to follow
   * @param targetPoseSupplier Optional supplier for a target pose (useful for dynamic targets)
   * @param mirror Whether to mirror the trajectory across the field
   * @return A command that follows the trajectory
   */
  public static DriveTrajectory createTrajectoryCommand(
      CommandSwerveDrivetrain drive,
      HolonomicTrajectory trajectory,
      Supplier<Pose2d> targetPoseSupplier,
      boolean mirror) {
    return new DriveTrajectory(drive, trajectory, targetPoseSupplier, mirror);
  }

  /**
   * Creates a command that aims the robot at a specific point while following a trajectory.
   *
   * @param trajectoryCommand The trajectory command to modify
   * @param targetPoseSupplier Supplier for the pose to aim at
   * @return A command that overrides the trajectory rotation to aim at the target
   */
  public static Command aimAtTarget(
      DriveTrajectory trajectoryCommand, Supplier<Pose2d> targetPoseSupplier) {
    return Commands.run(
            () -> {
              Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;
              Pose2d targetPose = targetPoseSupplier.get();

              // Calculate angle from robot to target
              Rotation2d aimAngle =
                  new Rotation2d(
                      targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());

              trajectoryCommand.setOverrideRotation(Optional.of(aimAngle));
            })
        .finallyDo(() -> trajectoryCommand.setOverrideRotation(Optional.empty()));
  }

  /**
   * Checks if the robot has crossed a specified X coordinate on the field.
   *
   * @param x The X coordinate to check
   * @param movingPositive Whether the robot is expected to be moving in the positive X direction
   * @return Whether the robot has crossed the specified X coordinate
   */
  public static boolean hasRobotCrossedX(double x, boolean movingPositive) {
    double robotX = RobotContainer.drivetrain.getState().Pose.getX();
    return movingPositive ? robotX >= x : robotX <= x;
  }

  /**
   * Checks if the robot has crossed a specified Y coordinate on the field.
   *
   * @param y The Y coordinate to check
   * @param movingPositive Whether the robot is expected to be moving in the positive Y direction
   * @return Whether the robot has crossed the specified Y coordinate
   */
  public static boolean hasRobotCrossedY(double y, boolean movingPositive) {
    double robotY = RobotContainer.drivetrain.getState().Pose.getY();
    return movingPositive ? robotY >= y : robotY <= y;
  }

  /**
   * Mirrors a pose across the field's center line. Useful for symmetrical field elements.
   *
   * @param pose The original pose
   * @return The mirrored pose
   */
  public static Pose2d mirrorPose(Pose2d pose) {
    // Mirror across Y axis by negating X and adjusting rotation
    return new Pose2d(
        pose.getX(),
        -pose.getY(),
        new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
  }

  /**
   * Creates a command that waits until the robot is within a certain distance of a target pose.
   *
   * @param targetPose The target pose
   * @param distanceMeters The distance threshold in meters
   * @return A command that completes when the robot is within the specified distance
   */
  public static Command waitUntilWithinDistance(Pose2d targetPose, double distanceMeters) {
    return Commands.waitUntil(
        () -> {
          Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
          double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
          return distance <= distanceMeters;
        });
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.trajectory.HolonomicTrajectory;
import frc.robot.util.AllianceFlipUtil;
import lombok.RequiredArgsConstructor;

/**
 * The AutoBuilder class provides a framework for creating autonomous routines.
 *
 * <p>This class is responsible for:
 *
 * <ul>
 *   <li>Creating autonomous command sequences
 *   <li>Managing trajectory generation and execution
 *   <li>Handling alliance-specific coordinate transformations
 *   <li>Composing complex sequences from simpler command building blocks
 * </ul>
 *
 * <p>To use this class:
 *
 * <ol>
 *   <li>Extend this class or use it directly to create autonomous routines
 *   <li>Define methods that return Command objects representing autonomous sequences
 *   <li>Use the utility methods to build trajectories and command sequences
 *   <li>Register the commands with the autonomous chooser in RobotContainer
 * </ol>
 */
@RequiredArgsConstructor
public class AutoBuilder {
  /** The drive subsystem used for trajectory following */
  public final CommandSwerveDrivetrain drive;

  /**
   * Creates a command that resets robot odometry to a specified pose.
   *
   * @param pose The pose to reset to (will be flipped based on alliance)
   * @return A command that resets the odometry
   */
  public Command resetOdometryCommand(Pose2d pose) {
    return Commands.runOnce(() -> drive.resetPose(AllianceFlipUtil.apply(pose)));
  }

  /**
   * Creates a command to follow a trajectory.
   *
   * @param trajectoryName The name of the trajectory resource to follow
   * @return A command sequence that resets odometry and follows the trajectory
   */
  public Command followTrajectory(String trajectoryName) {
    HolonomicTrajectory trajectory = new HolonomicTrajectory(trajectoryName);

    return Commands.sequence(
        resetOdometryCommand(trajectory.getStartPose()), new DriveTrajectory(drive, trajectory));
  }

  /**
   * Creates a command to drive to a specific pose on the field.
   *
   * @param targetPose The pose to drive to (will be flipped based on alliance)
   * @return A command that drives to the specified pose
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    return new DriveToPose(drive, () -> AllianceFlipUtil.apply(targetPose));
  }

  /**
   * Creates a command sequence from a series of poses. The robot will drive to each pose in
   * sequence.
   *
   * @param poses The sequence of poses to visit
   * @return A command that drives through all poses in sequence
   */
  public Command followPoseSequence(Pose2d... poses) {
    if (poses.length == 0) {
      return Commands.none();
    }

    SequentialCommandGroup sequence = new SequentialCommandGroup();
    sequence.addCommands(resetOdometryCommand(poses[0]));

    for (int i = 0; i < poses.length; i++) {
      sequence.addCommands(driveToPoseCommand(poses[i]));
    }

    return sequence;
  }

  /**
   * Transforms a pose by a specified delta. Useful for creating waypoints relative to a starting
   * position.
   *
   * @param basePose The base pose
   * @param deltaX X offset in meters
   * @param deltaY Y offset in meters
   * @param deltaRotation Rotation offset in radians
   * @return The transformed pose
   */
  protected Pose2d transformPose(
      Pose2d basePose, double deltaX, double deltaY, double deltaRotation) {
    return basePose.plus(
        new Transform2d(new Translation2d(deltaX, deltaY), new Rotation2d(deltaRotation)));
  }

  /**
   * Example autonomous routine template. Replace this with your own autonomous routines.
   *
   * @return A command representing a simple autonomous routine
   */
  public Command exampleAutoRoutine() {
    // Define starting position
    Pose2d startPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(0)));

    // Create waypoints
    Pose2d midPose = transformPose(startPose, 2.0, 0.0, 0.0);
    Pose2d endPose = transformPose(midPose, 1.0, 1.0, Math.toRadians(90));

    // Build the command sequence
    return Commands.sequence(
        resetOdometryCommand(startPose),
        driveToPoseCommand(midPose),
        Commands.waitSeconds(0.5), // Pause briefly
        driveToPoseCommand(endPose));
  }

  /**
   * Creates a do-nothing autonomous routine.
   *
   * @return A command that does nothing
   */
  public Command doNothingAuto() {
    return Commands.none();
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Utility class for transforming field coordinates between red and blue alliances. Supports both 2D
 * and 3D poses for maximum flexibility. Used for the 2025 Reefscape field.
 */
public class FlipField {
  // Field dimensions for 2025 Reefscape
  // If the middle Y is 6.08, then the full width would be approximately 12.16 meters
  public static final double FIELD_LENGTH_METERS = 17.53; // 54'1"
  // public static final double FIELD_WIDTH_METERS = 12.16; // Calculated from center Y position

  // Field center point
  public static final double FIELD_CENTER_X = 8.765; // 8.24 meters

  // public static final double FIELD_CENTER_Y = FIELD_WIDTH_METERS / 2.0; // 6.08 meters

  /**
   * Flips a Pose3d around the field center line. This method mirrors the X coordinate across the
   * vertical centerline of the field, keeps the Y coordinate the same, and adjusts the rotation
   * appropriately. The Z coordinate remains unchanged.
   *
   * @param pose The original pose (typically from red alliance)
   * @return A new Pose3d flipped to the other side of the field (typically for blue alliance)
   */
  public static Pose3d flipPose(Pose3d pose) {
    // Calculate the mirrored X position (flip across vertical center line)
    double flippedX = FIELD_LENGTH_METERS - pose.getX();

    // Y and Z positions remain the same
    double flippedY = pose.getY();
    double flippedZ = pose.getZ();

    // Flip the rotation properly for mirroring across the centerline
    Rotation3d originalRotation = pose.getRotation();

    // When mirroring across the centerline:
    // 1. Roll (X-axis rotation) should be negated
    // 2. Pitch (Y-axis rotation) remains the same
    // 3. Yaw (Z-axis rotation) needs to be flipped to its supplement (Math.PI - yaw)
    Rotation3d flippedRotation =
        new Rotation3d(
            -originalRotation.getX(), // Negate roll when mirroring
            originalRotation.getY(), // Pitch remains the same
            Math.PI - originalRotation.getZ() // Yaw gets mirrored (using supplement angle)
            );

    return new Pose3d(flippedX, flippedY, flippedZ, flippedRotation);
  }

  /**
   * Flips a Pose2d around the field center line. This method mirrors the X coordinate across the
   * vertical centerline of the field, keeps the Y coordinate the same, and adjusts the rotation
   * appropriately.
   *
   * @param pose The original pose (typically from red alliance)
   * @return A new Pose2d flipped to the other side of the field (typically for blue alliance)
   */
  public static Pose2d flipPose(Pose2d pose) {
    // Calculate the mirrored X position (flip across vertical center line)
    double flippedX = FIELD_LENGTH_METERS - pose.getX();

    // Y position remains the same
    double flippedY = pose.getY();

    // Flip the rotation (mirror across centerline)
    double originalAngle = pose.getRotation().getRadians();
    Rotation2d flippedRotation = new Rotation2d(Math.PI - originalAngle);

    return new Pose2d(flippedX, flippedY, flippedRotation);
  }

  /**
   * Alternative method name for Pose3d flipping. This is an alias for flipPose() for backward
   * compatibility.
   *
   * @param pose The original pose (typically from red alliance)
   * @return A new Pose3d flipped to the other side of the field (typically for blue alliance)
   */
  public static Pose3d FieldFlip(Pose3d pose) {
    return flipPose(pose);
  }

  /**
   * Alternative method name for Pose2d flipping. This is an alias for flipPose() for backward
   * compatibility.
   *
   * @param pose The original pose (typically from red alliance)
   * @return A new Pose2d flipped to the other side of the field (typically for blue alliance)
   */
  public static Pose2d FieldFlip(Pose2d pose) {
    return flipPose(pose);
  }

  /**
   * Determines if a Pose3d should be flipped based on current alliance color. If alliance is red,
   * the pose is returned unchanged. If alliance is blue, the pose is flipped.
   *
   * @param pose The original pose (always stored in red alliance frame)
   * @return The pose, potentially flipped if on blue alliance
   */
  public static Pose3d flipIfBlue(Pose3d pose) {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    if (alliance.isPresent()
        && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      return flipPose(pose);
    }
    return pose;
  }

  /**
   * Determines if a Pose2d should be flipped based on current alliance color. If alliance is red,
   * the pose is returned unchanged. If alliance is blue, the pose is flipped.
   *
   * @param pose The original pose (always stored in red alliance frame)
   * @return The pose, potentially flipped if on blue alliance
   */
  public static Pose2d flipIfBlue(Pose2d pose) {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    if (alliance.isPresent()
        && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      return flipPose(pose);
    }
    return pose;
  }
}

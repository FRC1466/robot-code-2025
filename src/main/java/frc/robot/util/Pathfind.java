// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.PathfindConstants;
import java.io.IOException;
import java.text.ParseException;
import java.util.Optional;

public class Pathfind {
  PathPlannerPath path;
  PathConstraints constraints;
  static Command redPathfindingCommand;
  static Command bluePathfindingCommand;
  public static Pose2d[] redAveragePoses = calculateAverageRedReefPoses();
  public static Pose2d[] blueAveragePoses = calculateAverageBlueReefPoses();

  // sendable chooser for pathfinding testing
  static Command TestPathfindingCommand;
  public static SendableChooser<Pose2d> testPoseChooser = new SendableChooser<>();

  // String bestPath = "Tag 10 to G";
  // obj 0 is left, obj 1 is right
  // 6,7,8,9,10,11

  // Removed local redTargetPoseReef and blueTargetPoseReef arrays

  // public Pose2d targetPose = new Pose2d(11.638, 3.863, Rotation2d.fromDegrees(0));

  public Pathfind(RobotContainer robotContainer) throws IOException, ParseException {
    this.robotContainer = robotContainer;

    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path.
    constraints =
        new PathConstraints(5, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  @SuppressWarnings("unused")
  private final RobotContainer robotContainer;

  public Command getPathfindingCommandReef(int targetLeftOrRight, int closestTag) {
    int currentClosestTag = closestTag;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            PathfindConstants.redTargetPoseReef[currentClosestTag][targetLeftOrRight],
            constraints,
            0.0);

    // Use FlipField.flipPose() to create blue alliance pose from red alliance pose
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                PathfindConstants.redTargetPoseReef[currentClosestTag][targetLeftOrRight]),
            constraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandReefL4(int targetLeftOrRight, int closestTag) {
    int currentClosestTag = closestTag;
    Pose2d targetPose = PathfindConstants.redTargetPoseReef[currentClosestTag][targetLeftOrRight];

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            new Pose2d(
                targetPose.getX() - ((.05) * Math.cos(targetPose.getRotation().getRadians())),
                targetPose.getY() - ((.05) * Math.sin(targetPose.getRotation().getRadians())),
                targetPose.getRotation()),
            constraints,
            0.0);

    // Use FlipField.flipPose() to create blue alliance pose from red alliance pose
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                new Pose2d(
                    targetPose.getX() - ((.05) * Math.cos(targetPose.getRotation().getRadians())),
                    targetPose.getY() - ((.05) * Math.sin(targetPose.getRotation().getRadians())),
                    targetPose.getRotation())),
            constraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandStation(int closestStation) {
    int currentClosestStation = closestStation;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            PathfindConstants.redTargetPoseStation[currentClosestStation], constraints, 0.0);

    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(PathfindConstants.redTargetPoseStation[currentClosestStation]),
            constraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandAlgae(int closestTag) {
    int currentClosestTag = closestTag;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(redAveragePoses[currentClosestTag], constraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(blueAveragePoses[currentClosestTag], constraints, 0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandBarge(double yBarge) {
    // Flip how barge pose is clamped over the middle of field (The field is 8.052 M)
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
      yBarge = MathUtil.clamp(yBarge, 0.5, 3.5);
    } else {
      yBarge = MathUtil.clamp(yBarge, 4.552, 7.552);
    }
    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            new Pose2d(
                new Translation2d(PathfindConstants.redTargetPoseXBarge, yBarge),
                new Rotation2d(Math.PI)),
            constraints,
            0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                new Pose2d(
                    new Translation2d(PathfindConstants.redTargetPoseXBarge, yBarge),
                    new Rotation2d(Math.PI))),
            constraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  // Calculate average of left and right reef poses for red alliance
  private static Pose2d[] calculateAverageRedReefPoses() {
    Pose2d[] averagePoses = new Pose2d[PathfindConstants.redTargetPoseReef.length];
    for (int i = 0; i < PathfindConstants.redTargetPoseReef.length; i++) {
      Pose2d left = PathfindConstants.redTargetPoseReef[i][0];
      Pose2d right = PathfindConstants.redTargetPoseReef[i][1];
      averagePoses[i] =
          new Pose2d(
              (left.getX() + right.getX()) / 2,
              (left.getY() + right.getY()) / 2,
              left.getRotation() // Using left pose rotation as reference
              );
    }
    return averagePoses;
  }

  // Calculate average of left and right reef poses for blue alliance
  private static Pose2d[] calculateAverageBlueReefPoses() {
    // Get the average red poses first
    Pose2d[] averageRedPoses = calculateAverageRedReefPoses();

    // Then flip each average pose to get the blue alliance equivalents
    Pose2d[] averageBluePoses = new Pose2d[averageRedPoses.length];
    for (int i = 0; i < averageRedPoses.length; i++) {
      averageBluePoses[i] = FlipField.FieldFlip(averageRedPoses[i]);
    }

    return averageBluePoses;
  }

  public Command getPathfindingCommandProcessor() {
    redPathfindingCommand =
        AutoBuilder.pathfindToPose(PathfindConstants.redTargetPoseProcessor, constraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(MirrorUtil.apply(PathfindConstants.redTargetPoseProcessor)),
            constraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }
}

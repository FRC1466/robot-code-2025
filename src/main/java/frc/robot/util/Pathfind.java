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
import frc.robot.constants.FieldConstants.CoralStation;
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
    return getPathfindingCommandReef(targetLeftOrRight, closestTag, constraints);
  }

  public Command getPathfindingCommandReef(
      int targetLeftOrRight, int closestTag, PathConstraints customConstraints) {
    int currentClosestTag = closestTag;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                PathfindConstants.blueTargetPoseReef[currentClosestTag][targetLeftOrRight]),
            customConstraints,
            0.0);

    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                PathfindConstants.blueTargetPoseReef[currentClosestTag][targetLeftOrRight]),
            customConstraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandReefL4(int targetLeftOrRight, int closestTag) {
    return getPathfindingCommandReefL4(targetLeftOrRight, closestTag, constraints);
  }

  public Command getPathfindingCommandReefL4(
      int targetLeftOrRight, int closestTag, PathConstraints customConstraints) {
    int currentClosestTag = closestTag;
    Pose2d targetPose = PathfindConstants.blueTargetPoseReef[currentClosestTag][targetLeftOrRight];

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            new Pose2d(
                targetPose.getX() - ((.05) * Math.cos(targetPose.getRotation().getRadians())),
                targetPose.getY() - ((.05) * Math.sin(targetPose.getRotation().getRadians())),
                targetPose.getRotation()),
            customConstraints,
            0.0);

    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                new Pose2d(
                    targetPose.getX() - ((.05) * Math.cos(targetPose.getRotation().getRadians())),
                    targetPose.getY() - ((.05) * Math.sin(targetPose.getRotation().getRadians())),
                    targetPose.getRotation())),
            customConstraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandStation(int closestStation) {
    return getPathfindingCommandStation(closestStation, constraints);
  }

  public Command getPathfindingCommandStation(
      int closestStation, PathConstraints customConstraints) {
    int currentClosestStation = closestStation;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            PathfindConstants.redTargetPoseStation[currentClosestStation], customConstraints, 0.0);

    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(PathfindConstants.redTargetPoseStation[currentClosestStation]),
            customConstraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandAlgae(int closestTag) {
    return getPathfindingCommandAlgae(closestTag, constraints);
  }

  public Command getPathfindingCommandAlgae(int closestTag, PathConstraints customConstraints) {
    int currentClosestTag = closestTag;

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(redAveragePoses[currentClosestTag], customConstraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(blueAveragePoses[currentClosestTag], customConstraints, 0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandBarge(double yBarge) {
    return getPathfindingCommandBarge(yBarge, constraints);
  }

  public Command getPathfindingCommandBarge(double yBarge, PathConstraints customConstraints) {
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
                new Rotation2d(0)),
            customConstraints,
            0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(
                new Pose2d(
                    new Translation2d(PathfindConstants.redTargetPoseXBarge, yBarge),
                    new Rotation2d(0))),
            customConstraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  public Command getPathfindingCommandProcessor() {
    return getPathfindingCommandProcessor(constraints);
  }

  public Command getPathfindingCommandProcessor(PathConstraints customConstraints) {
    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            PathfindConstants.redTargetPoseProcessor, customConstraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            FlipField.FieldFlip(MirrorUtil.apply(PathfindConstants.redTargetPoseProcessor)),
            customConstraints,
            0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  /**
   * Get the closest point from a sliding set of points along the coral station face.
   *
   * @param currentPose The current pose of the robot.
   * @return The closest pose aligned with the coral station.
   */
  public Pose2d getClosestCoralStationPose(Pose2d currentPose) {
    Translation2d robotPosition = currentPose.getTranslation();

    // Generate a sliding set of points along the coral station face
    int numPoints = 9; // Number of points (indents) along the station face
    Pose2d[] stationPoints = new Pose2d[numPoints];
    double startY = CoralStation.rightCenterFace.getY();
    double endY = CoralStation.leftCenterFace.getY();
    double xPosition = CoralStation.rightCenterFace.getX(); // X-coordinate remains constant
    double spacing = (endY - startY) / (numPoints - 1);

    for (int i = 0; i < numPoints; i++) {
      double yPosition = startY + i * spacing;
      stationPoints[i] =
          new Pose2d(xPosition, yPosition, CoralStation.rightCenterFace.getRotation());
    }

    // Flip station points for red alliance
    boolean isRedAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
    if (isRedAlliance) {
      for (int i = 0; i < stationPoints.length; i++) {
        stationPoints[i] = FlipField.FieldFlip(stationPoints[i]);
      }
    }

    // Find the closest point to the robot's current position
    Pose2d closestPose = null;
    double closestDistance = Double.MAX_VALUE;

    for (Pose2d stationPose : stationPoints) {
      double distance = robotPosition.getDistance(stationPose.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = stationPose;
      }
    }

    return closestPose;
  }

  /**
   * Example method to use the closest coral station pose for pathfinding.
   *
   * @param currentPose The current pose of the robot.
   * @return The pathfinding command to the closest coral station pose.
   */
  public Command getPathfindingCommandStation(Pose2d currentPose) {
    Pose2d targetPose = getClosestCoralStationPose(currentPose);

    if (targetPose == null) {
      throw new IllegalStateException("No valid coral station pose found.");
    }

    // Generate pathfinding command to the target pose
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  // Calculate average of left and right reef poses for red alliance
  private static Pose2d[] calculateAverageRedReefPoses() {
    Pose2d[] averagePoses = new Pose2d[PathfindConstants.blueTargetPoseReef.length];
    for (int i = 0; i < PathfindConstants.blueTargetPoseReef.length; i++) {
      Pose2d left = PathfindConstants.blueTargetPoseReef[i][0];
      Pose2d right = PathfindConstants.blueTargetPoseReef[i][1];
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
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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
        new PathConstraints(5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));
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
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            PathfindConstants.blueTargetPoseReef[currentClosestTag][targetLeftOrRight],
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
            PathfindConstants.blueTargetPoseStation[currentClosestStation], constraints, 0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }

  // Calculate average of left and right reef poses
  private Pose2d[] calculateAverageRedReefPoses() {
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

  private Pose2d[] calculateAverageBlueReefPoses() {
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

  public Command getPathfindingCommandAlgae(int closestTag) {
    int currentClosestTag = closestTag;

    // Get dynamically calculated average poses
    Pose2d[] redAveragePoses = calculateAverageRedReefPoses();
    Pose2d[] blueAveragePoses = calculateAverageBlueReefPoses();

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(redAveragePoses[currentClosestTag], constraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(blueAveragePoses[currentClosestTag], constraints, 0.0);

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance =
        allianceOptional.orElse(Alliance.Blue); // choose default alliance if not present
    return alliance == Alliance.Red ? redPathfindingCommand : bluePathfindingCommand;
  }
}

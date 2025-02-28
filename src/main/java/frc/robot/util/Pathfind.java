// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.text.ParseException;
import org.littletonrobotics.junction.Logger;

public class Pathfind {
  PathPlannerPath path;
  PathConstraints constraints;
  static Command redPathfindingCommand;
  static Command bluePathfindingCommand;

  // sendable chooser for pathfinding testing
  static Command TestPathfindingCommand;
  public static SendableChooser<Pose2d> testPoseChooser = new SendableChooser<>();

  /*private void setupPoseChooser() {
    testPoseChooser.setDefaultOption("Tag 10 to G", redTargetPose[5][0]);
    testPoseChooser.addOption("Tag 9 to G", redTargetPose[4][0]);
    testPoseChooser.addOption("Tag 8 to G", redTargetPose[3][0]);
    testPoseChooser.addOption("Tag 7 to G", redTargetPose[2][0]);
    testPoseChooser.addOption("Tag 6 to G", redTargetPose[1][0]);
    testPoseChooser.addOption("Tag 5 to G", redTargetPose[0][0]);

    // add the chooser to the dashboard
    SmartDashboard.putData(testPoseChooser);
  }*/

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
      new Pose2d(12.301, 5.237, Rotation2d.fromDegrees(-60))
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

  // DO NOT USE - NEEDS TO BE FIXED
  public Pose2d[][] blueTargetPoseTransformed = {
    {
      // 4.022; 300° becomes -60°
      new Pose2d(4.022, 2.837, Rotation2d.fromDegrees(-60)),
      // 3.719; 300° becomes -60°
      new Pose2d(3.719, 3.003, Rotation2d.fromDegrees(-60))
    },
    {
      // 3.225; 0° remains 0°
      new Pose2d(3.225, 3.860, Rotation2d.fromDegrees(0)),
      // 3.225; 0° remains 0°
      new Pose2d(3.225, 4.190, Rotation2d.fromDegrees(0))
    },
    {
      // 3.732; 60° remains 60°
      new Pose2d(3.732, 5.044, Rotation2d.fromDegrees(60)),
      // 4.012; 60° remains 60°
      new Pose2d(4.012, 5.216, Rotation2d.fromDegrees(60))
    },
    {
      // 5.011; 120° remains 120°
      new Pose2d(5.011, 5.237, Rotation2d.fromDegrees(120)),
      // 5.271; 120° remains 120°
      new Pose2d(5.271, 12.301, Rotation2d.fromDegrees(120))
    },
    {
      // 5.877; 180° remains 180°
      new Pose2d(5.877, 4.169, Rotation2d.fromDegrees(180)),
      // 5.877; 180° remains 180°
      new Pose2d(5.877, 3.860, Rotation2d.fromDegrees(180))
    },
    {
      // 5.289; 240° becomes -120°
      new Pose2d(5.289, 2.998, Rotation2d.fromDegrees(-120)),
      // 5.018; 240° becomes -120°
      new Pose2d(5.018, 2.850, Rotation2d.fromDegrees(-120))
    }
  };

  // public Pose2d targetPose = new Pose2d(11.638, 3.863, Rotation2d.fromDegrees(0));

  public Pathfind(RobotContainer robotContainer) throws IOException, ParseException {
    this.robotContainer = robotContainer;
    // setupPoseChooser();
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

  }

  private final RobotContainer robotContainer;

  public Command getPathfindingCommand(int targetLeftOrRight, int closestTag) {
    int currentClosestTag = closestTag;
    Logger.recordOutput("closest tag in pathfind", currentClosestTag);
    Logger.recordOutput("Target Point", redTargetPose[currentClosestTag][targetLeftOrRight]);

    redPathfindingCommand =
        AutoBuilder.pathfindToPose(
            redTargetPose[currentClosestTag][targetLeftOrRight], constraints, 0.0);
    bluePathfindingCommand =
        AutoBuilder.pathfindToPose(
            blueTargetPoseTransformed[currentClosestTag][targetLeftOrRight], constraints, 0.0);
    // Pose2d selectedPose = testPoseChooser.getSelected();
    // TestPathfindingCommand = AutoBuilder.pathfindToPose(selectedPose, constraints, 0.0);

    return DriverStation.getAlliance().get() == Alliance.Red
        ? redPathfindingCommand
        : bluePathfindingCommand;
  }
}

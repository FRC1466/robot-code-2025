// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.text.ParseException;

public class Pathfind {
  PathPlannerPath path;
  PathConstraints constraints;
  static Command pathfindingCommand;
  String bestPath = "Tag 10 to G";

  public Pathfind() throws IOException, ParseException {
    // Load the path we want to pathfind to and follow
    try {
      path = PathPlannerPath.fromPathFile(bestPath);
    } catch (FileVersionException | IOException | org.json.simple.parser.ParseException e) {
      System.err.println("Error loading path: " + e.getMessage());
      e.printStackTrace();
    }

    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path.
    constraints =
        new PathConstraints(.25, .25, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public static Command getPathfindingCommand() {
    return pathfindingCommand;
  }
}

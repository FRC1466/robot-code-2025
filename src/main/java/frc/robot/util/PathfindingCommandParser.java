// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class PathfindingCommandParser {
  private static PathfindingAutoCommands pathfindingAutoCommands =
      RobotContainer.m_PathfindingAutoCommands;

  // Data class to hold coordinate info.
  public static class Coordinate {
    public int tag;
    public int side;
    public int level;

    public Coordinate(int tag, int side, int level) {
      this.tag = tag;
      this.side = side;
      this.level = level;
    }

    @Override
    public String toString() {
      return "Reef " + tag + ", Side " + side + ", Level " + level;
    }
  }

  public static Command scoreCoral(Coordinate coord) {
    Logger.recordOutput("PathfindingParser", "Score Coral");
    Logger.recordOutput("PathfindingParser", coord.toString());
    return pathfindingAutoCommands.l321AutoFreaktory(coord.tag, coord.side, coord.level);
  }

  public static Command scoreCoralL4(Coordinate coord) {
    Logger.recordOutput("PathfindingParser", "Score Coral L4");
    Logger.recordOutput("PathfindingParser", coord.toString());
    return pathfindingAutoCommands.l4AutoFreaktory(coord.tag, coord.side);
  }

  public static Command intakeStation() {
    Logger.recordOutput("PathfindingParser", "Station Intake");
    return pathfindingAutoCommands.stationAutoFreaktory();
  }

  public static Command intakeAlgae(int tag) {
    Logger.recordOutput("PathfindingParser", "Algae Intake");
    Logger.recordOutput("PathfindingParser", "Tag: " + tag);
    return pathfindingAutoCommands.algaeAutoFreaktory(tag);
  }

  public static Command scoreProcessor() {
    Logger.recordOutput("PathfindingParser", "Processor Score");
    return Commands.runOnce(() -> pathfindingAutoCommands.processorAutoFreaktory());
  }

  public static Command scoreBarge() {
    Logger.recordOutput("PathfindingParser", "Barge Score");
    return Commands.runOnce(() -> pathfindingAutoCommands.bargeAutoFreaktory());
  }

  /**
   * Parses the given command string and schedules a single sequential command that executes through
   * all tokens once.
   *
   * <p>Expected token types: • Coordinate tokens: "reef-side-level" (e.g., "5-0-4") – Updates the
   * current coordinate. – For level 2 or 3, calls scoreCoral(); for level 4, calls scoreCoralL4().
   * • "S": perform station intake. • Tokens matching "[number]-A" (e.g., "4-A") : perform algae
   * intake (using the number as tag). • "P": perform processor score. • "B": perform barge score.
   *
   * <p>The parser handles arbitrarily many tokens.
   *
   * @param commandString the full command string (e.g., "5-0-4--S--5-1-3--4-A")
   */
  public static void parseCommandString(String commandString) {
    String timestamp = String.format("%.2f", Timer.getFPGATimestamp());
    Logger.recordOutput("AutoStatus", "Starting command: " + commandString);
    Logger.recordOutput("PathfindingConsole", timestamp + " - Starting command: " + commandString);

    // Split by delimiter "--" (tokens can be arbitrarily many)
    String[] tokens = commandString.split("--");
    for (int i = 0; i < tokens.length; i++) {
      tokens[i] = tokens[i].toUpperCase();
    }
    Command sequentialCommands = Commands.none();
    Coordinate currentCoord = null;

    for (String rawToken : tokens) {
      String token = rawToken.trim();
      // Log the raw token along with its length for debugging.
      Logger.recordOutput(
          "PathfindingConsole", "Token received: '" + token + "' (length=" + token.length() + ")");
      if (token.isEmpty()) continue;

      timestamp = String.format("%.2f", Timer.getFPGATimestamp());
      Logger.recordOutput("PathfindingConsole", timestamp + " - Processing token: " + token);

      // Fix: Use independent if blocks rather than else-if chains
      // Coordinate token: should match three numbers separated by dashes.
      if (token.matches("\\d+\\s*-\\s*\\d+\\s*-\\s*\\d+")) {
        String[] parts = token.split("\\s*-\\s*");
        try {
          int reef = Integer.parseInt(parts[0]);
          int side = Integer.parseInt(parts[1]);
          int level = Integer.parseInt(parts[2]);
          currentCoord = new Coordinate(reef, side, level);
          Logger.recordOutput(
              "PathfindingConsole",
              timestamp
                  + " - Parsed coordinate: Reef="
                  + reef
                  + ", Side="
                  + side
                  + ", Level="
                  + level);

          switch (level) {
            case 2, 3:
              final Coordinate coord23 = currentCoord;
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole",
                                    execTimestamp + " - EXECUTING score (L2/L3) at " + coord23);
                                Logger.recordOutput(
                                    "AutoStatus", "Executing score (L2/L3) at " + coord23);
                              })
                          .andThen(scoreCoral(coord23)));
            case 4:
              final Coordinate coordL4 = currentCoord;
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole",
                                    execTimestamp + " - EXECUTING score (L4) at " + coordL4);
                                Logger.recordOutput(
                                    "AutoStatus", "Executing score (L4) at " + coordL4);
                              })
                          .andThen(scoreCoralL4(coordL4)));
            default:
              Logger.recordOutput(
                  "PathfindingConsole", timestamp + " - Level not supported: " + level);
          }
        } catch (NumberFormatException e) {
          Logger.recordOutput(
              "PathfindingConsole", timestamp + " - Invalid coordinate token: " + token);
        }
      }

      // Algae intake token: allow optional spaces around the dash (pattern like "4-A" or "4 - A")
      if (token.matches("\\d+\\s*-\\s*A")) { // Changed from else if to if
        String[] parts = token.split("\\s*-\\s*");
        try {
          int tag = Integer.parseInt(parts[0]);
          Logger.recordOutput(
              "PathfindingConsole", timestamp + " - Will perform algae intake at tag " + tag);
          sequentialCommands =
              sequentialCommands.andThen(
                  Commands.runOnce(
                          () -> {
                            String execTimestamp = String.format("%.2f", Timer.getFPGATimestamp());
                            Logger.recordOutput(
                                "PathfindingConsole",
                                execTimestamp + " - EXECUTING algae intake at tag " + tag);
                            Logger.recordOutput(
                                "AutoStatus", "Executing algae intake at tag " + tag);
                          })
                      .andThen(intakeAlgae(tag)));
        } catch (NumberFormatException e) {
          Logger.recordOutput("PathfindingConsole", timestamp + " - Invalid algae token: " + token);
        }
      }

      // Single-letter tokens.
      if (token.length() == 1) { // Changed from else to if
        String action = token.toUpperCase();
        switch (action) {
          case "S":
            Logger.recordOutput("PathfindingConsole", timestamp + " - Will perform station intake");
            sequentialCommands =
                sequentialCommands.andThen(
                    Commands.runOnce(
                            () -> {
                              String execTimestamp =
                                  String.format("%.2f", Timer.getFPGATimestamp());
                              Logger.recordOutput(
                                  "PathfindingConsole",
                                  execTimestamp + " - EXECUTING station intake");
                              Logger.recordOutput("AutoStatus", "Executing station intake");
                            })
                        .andThen(intakeStation()));
            /*case "P":
              Logger.recordOutput(
                  "PathfindingConsole", timestamp + " - Will perform processor score");
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole",
                                    execTimestamp + " - EXECUTING processor score");
                                Logger.recordOutput("AutoStatus", "Executing processor score");
                              })
                          .andThen(scoreProcessor()));
            case "B":
              Logger.recordOutput("PathfindingConsole", timestamp + " - Will perform barge score");
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole", execTimestamp + " - EXECUTING barge score");
                                Logger.recordOutput("AutoStatus", "Executing barge score");
                              })
                          .andThen(scoreBarge()));*/
          default:
            Logger.recordOutput("PathfindingConsole", timestamp + " - Unknown token: " + token);
        }
      }
    }

    timestamp = String.format("%.2f", Timer.getFPGATimestamp());
    Logger.recordOutput(
        "PathfindingConsole", timestamp + " - Command parsing complete, scheduling execution");
    Logger.recordOutput("AutoStatus", "Command parsing complete, scheduling execution");

    sequentialCommands.schedule();
  }
}

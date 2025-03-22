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
  public static Command parseCommandString(String commandString) {
    Logger.recordOutput("AutoStatus", "Starting command: " + commandString);
    Logger.recordOutput("PathfindingConsole", "Starting command: " + commandString);

    // Split by delimiter "--" (tokens can be arbitrarily many)
    String[] tokens = commandString.split("--");
    Logger.recordOutput("tokens", "Tokens: " + String.join(", ", tokens));
    Command sequentialCommands = Commands.none();
    Coordinate currentCoord = null;

    for (String rawToken : tokens) {
      String token = rawToken.trim();
      // Log the raw token along with its length for debugging.
      Logger.recordOutput(
          "PathfindingConsole", "Token received: '" + token + "' (length=" + token.length() + ")");
      if (token.isEmpty()) continue;
      Logger.recordOutput("PathfindingConsole", "Processing token: " + token);

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
              "Parsed coordinate: Reef=" + reef + ", Side=" + side + ", Level=" + level);

          switch (level) {
            case 2:
              final Coordinate coordL2 = currentCoord;
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole",
                                    execTimestamp + " - EXECUTING score at " + coordL2);
                                Logger.recordOutput("AutoStatus", "Executing score at " + coordL2);
                              })
                          .andThen(pathfindingAutoCommands.l321AutoFreaktory(reef, side, level)));
              continue;
            case 3:
              final Coordinate coordL3 = currentCoord;
              sequentialCommands =
                  sequentialCommands.andThen(
                      Commands.runOnce(
                              () -> {
                                String execTimestamp =
                                    String.format("%.2f", Timer.getFPGATimestamp());
                                Logger.recordOutput(
                                    "PathfindingConsole",
                                    execTimestamp + " - EXECUTING score at " + coordL3);
                                Logger.recordOutput("AutoStatus", "Executing score at " + coordL3);
                              })
                          .andThen(pathfindingAutoCommands.l321AutoFreaktory(reef, side, level)));
              continue;
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
                          .andThen(pathfindingAutoCommands.l4AutoFreaktory(reef, side)));
              continue;
            default:
              Logger.recordOutput("PathfindingConsole", "Level not supported: " + level);
              continue;
          }
        } catch (NumberFormatException e) {
          Logger.recordOutput("PathfindingConsole", "Invalid coordinate token: " + token);
          continue;
        }
      }

      // Algae intake token: allow optional spaces around the dash (pattern like "4-A" or "4 - A")
      if (token.matches("\\d+\\s*-\\s*A")) { // Changed from else if to if
        String[] parts = token.split("\\s*-\\s*");
        try {
          int tag = Integer.parseInt(parts[0]);
          Logger.recordOutput("PathfindingConsole", "Will perform algae intake at tag " + tag);
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
                      .andThen(pathfindingAutoCommands.algaeAutoFreaktory(tag)));
          continue;
        } catch (NumberFormatException e) {
          Logger.recordOutput("PathfindingConsole", "Invalid algae token: " + token);
          continue;
        }
      }

      // Single-letter tokens.
      if (token.length() == 1) { // Changed from else to if
        String action = token.toUpperCase();
        switch (action) {
          case "S":
            Logger.recordOutput("PathfindingConsole", "Will perform station intake");
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
                        .andThen(pathfindingAutoCommands.stationAutoFreaktory()));
            continue;
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
            Logger.recordOutput("PathfindingConsole", "Unknown token: " + token);
            continue;
        }
      }
    }
    Logger.recordOutput("PathfindingConsole", "Command parsing complete, scheduling execution");
    Logger.recordOutput("AutoStatus", "Command parsing complete, scheduling execution");

    sequentialCommands =
        sequentialCommands.andThen(
            RobotContainer.elevator
                .toBottom()
                .alongWith(
                    RobotContainer.rotaryPart
                        .coralScore()
                        .withTimeout(0.2)
                        .alongWith(RobotContainer.intake.stop())));
    return sequentialCommands;
  }
}

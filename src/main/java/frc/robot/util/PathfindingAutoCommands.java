// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import org.littletonrobotics.junction.Logger;

public class PathfindingAutoCommands {
  private final Pathfind m_pathfinder;
  private final Intake intake;
  private final Elevator elevator;
  private final RobotContainer robotContainer;

  public PathfindingAutoCommands(
      Pathfind pathfinder, Intake intake, Elevator elevator, RobotContainer robotContainer) {
    this.m_pathfinder = pathfinder;
    this.intake = intake;
    this.elevator = elevator;
    this.robotContainer = robotContainer;
  }

  public Command l4AutoFreaktory(int targetTag, int targetSide) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to first target (Tag " + targetTag + ")");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandReefL4(targetSide, targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.l4coralScore().withTimeout(.2),
        Commands.sequence(
            Commands.waitUntil(
                () -> {
                  boolean approaching =
                      robotContainer.armApproachingTarget(targetSide, targetTag, 2.5);
                  if (approaching) {
                    Logger.recordOutput("AutoStatus", "Starting elevator early for third target");
                  }
                  return approaching;
                }),
            elevator.toL4()),
        // Final positioning check
        Commands.waitUntil(
            () -> {
              boolean ready = robotContainer.armFieldReady(targetSide, 1);
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for first target position: "
                      + (ready && stopped ? "Ready" : "Not Ready"));
              return ready && stopped;
            }),
        // Start elevator while finalizing position
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Raising elevator to L4 for first target");
                  intake.coralHold();
                })),
        Commands.waitUntil(() -> elevator.getElevatorHeight() > 61),
        Commands.sequence(
            Commands.runOnce(
                () -> Logger.recordOutput("AutoStatus", "Outtaking coral at first target")),
            intake.outTake(),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> Logger.recordOutput("AutoStatus", "Stopping intake after first score")),
            intake.stop()));
  }

  public Command l321AutoFreaktory(int targetTag, int targetSide, int height) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to first target (Tag " + targetTag + ")");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandReef(targetSide, targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.coralScore().withTimeout(.2),
        Commands.sequence(
            Commands.waitUntil(
                () -> {
                  boolean approaching =
                      robotContainer.armApproachingTarget(targetSide, targetTag, 2.5);
                  if (approaching) {
                    Logger.recordOutput("AutoStatus", "Starting elevator early for third target");
                  }
                  return approaching;
                }),
            (height == 2 ? elevator.toL2() : elevator.toL3())),
        // Final positioning check
        Commands.waitUntil(
            () -> {
              boolean ready = robotContainer.armFieldReady(targetSide, 1);
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for first target position: "
                      + (ready && stopped ? "Ready" : "Not Ready"));
              return ready && stopped;
            }),
        // Start elevator while finalizing position
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput(
                      "AutoStatus", "Raising elevator to L" + height + " for target");
                })),
        switch (height) {
          case 2 -> Commands.waitUntil(() -> elevator.getElevatorHeight() > 12);
          case 3 -> Commands.waitUntil(() -> elevator.getElevatorHeight() > 28);
          default -> Commands.waitUntil(() -> elevator.getElevatorHeight() > 12);
        },
        Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("AutoStatus", "Outtaking coral at target")),
            intake.outTake(),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> Logger.recordOutput("AutoStatus", "Stopping intake after first score")),
            intake.stop()));
  }

  public Command stationAutoFreaktory() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput("AutoStatus", "Starting pathfinding to station");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder
                    .getPathfindingCommandStation(robotContainer.getClosestStation())
                    .schedule();
              }
            }),
        elevator.toBottom(),
        RobotContainer.rotaryPart.store().withTimeout(.2),
        // Wait for position at station
        Commands.waitUntil(
            () -> {
              boolean ready = robotContainer.coralIntakeReady();
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for station position: " + (ready && stopped ? "Ready" : "Not Ready"));
              return ready && stopped;
            }),
        // Intake sequence
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Starting intake at station");
                  intake.intake();
                }),
            Commands.waitSeconds(0.6),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Stopping intake after collection");
                  intake.stop();
                })));
  }

  public Command algaeAutoFreaktory(int targetTag) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to algae at tag " + targetTag);
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandAlgae(targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.algaeGrab().withTimeout(.2),
        Commands.sequence(
            Commands.waitUntil(
                () -> {
                  boolean approaching = robotContainer.armApproachingTarget(0, targetTag, 1.5);
                  if (approaching) {
                    Logger.recordOutput("AutoStatus", "Starting elevator early for algae");
                  }
                  return approaching;
                }),
            targetTag % 2 == 0
                ? RobotContainer.elevator.toL2Algae()
                : RobotContainer.elevator.toL3Algae()),
        // Final positioning check
        Commands.waitUntil(
            () -> {
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              return stopped;
            }),
        // Intake sequence
        Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("AutoStatus", "Grabbing algae")),
            intake.reverseIntake(),
            Commands.waitSeconds(0.8),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Holding algae after collection");
                  intake.algaeHold();
                })));
  }

  public void processorAutoFreaktory() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'processorAutoFreaktory'");
  }

  public void bargeAutoFreaktory() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'bargeAutoFreaktory'");
  }
}

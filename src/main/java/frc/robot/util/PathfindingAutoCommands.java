// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.RobotEnableValue;

public class PathfindingAutoCommands {
  private final Pathfind m_pathfinder;
  private final Intake intake;
  private final Elevator elevator;
  private final RobotContainer robotContainer;

  public boolean holdAlgae = false;
  public int algaeHeight = 0;

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
              holdAlgae = false;
              RobotContainer.algaeMode = false;
              RobotContainer.intake.stop();
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to target (Tag " + targetTag + ")");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandReefL4(targetSide, targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.coralScore().withTimeout(.1),
        elevator.toL4(),
        // Final positioning check
        Commands.waitUntil(
            () -> {
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for target position: " + (stopped ? "Ready" : "Not Ready"));
              return stopped;
            }),
        switch (Constants.getRobot()) {
          case COMPBOT -> Commands.waitUntil(() -> elevator.getElevatorHeight() > 61);
          default -> Commands.waitSeconds(0);
        },
        // Start elevator while finalizing position
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Holding Coral");
                  intake.coralHold();
                }),
            RobotContainer.rotaryPart.l4coralScore().withTimeout(.01)),
        Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("AutoStatus", "Outtaking coral at target")),
            intake.outTake(),
            switch (Constants.getRobot()) {
              case SIMBOT -> Commands.waitSeconds(0.5);
              case COMPBOT ->
                  Commands.waitUntil(() -> intake.getIntakeDistanceBool())
                      .andThen(waitSeconds(0.1));
              default -> Commands.waitSeconds(0.5);
            },
            Commands.runOnce(
                () -> Logger.recordOutput("AutoStatus", "Stopping intake after score")),
            intake.stop()),
        RobotContainer.rotaryPart.coralScore().withTimeout(.01));
  }

  public Command l321AutoFreaktory(int targetTag, int targetSide, int height) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              holdAlgae = false;
              RobotContainer.algaeMode = false;
              RobotContainer.intake.stop();
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to target (Tag " + targetTag + ")");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandReef(targetSide, targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.coralScore().withTimeout(.01),
        Commands.sequence(
            Commands.waitUntil(
                () -> {
                  boolean approaching =
                      robotContainer.armApproachingTarget(targetSide, targetTag, 2.5);
                  if (approaching) {
                    Logger.recordOutput("AutoStatus", "Starting elevator early for target");
                  }
                  return approaching;
                }),
            switch (height) {
              case 2 -> elevator.toL2();
              case 3 -> elevator.toL3();
              default -> elevator.toL2();
            }),
        // Final positioning check
        Commands.waitUntil(
            () -> {
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for target position: " + (stopped ? "Ready" : "Not Ready"));
              return stopped;
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
        waitUntil(() -> robotContainer.isDrivetrainStopped(.05)),
        RobotContainer.rotaryPart.coralScore().withTimeout(.01),
        Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("AutoStatus", "Outtaking coral at target")),
            intake.outTake(),
            switch (Constants.getRobot()) {
              case SIMBOT -> Commands.waitSeconds(0.5);
              case COMPBOT ->
                  Commands.waitUntil(() -> intake.getIntakeDistanceBool())
                      .andThen(waitSeconds(0.1));
              default -> Commands.waitSeconds(0.5);
            },
            Commands.runOnce(
                () -> Logger.recordOutput("AutoStatus", "Stopping intake after score")),
            intake.stop()),
        RobotContainer.rotaryPart.coralScore().withTimeout(.01));
  }

  public Command stationAutoFreaktory() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              holdAlgae = false;
              RobotContainer.algaeMode = false;
              RobotContainer.intake.stop();
              Logger.recordOutput("AutoStatus", "Starting pathfinding to station");
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder
                    .getPathfindingCommandStation(robotContainer.getClosestStation())
                    .schedule();
              }
            }),
        waitUntil(() -> robotContainer.armAlgaeReadyl2(2.5)),
        elevator.toBottom(),
        Commands.waitUntil(() -> elevator.getElevatorHeight() < 2),
        RobotContainer.rotaryPart.store().withTimeout(.01),
        // Wait for position at station
        Commands.waitUntil(
            () -> {
              boolean stopped = robotContainer.isDrivetrainStopped(0.05);
              Logger.recordOutput(
                  "AutoStatus",
                  "Waiting for station position: " + (stopped ? "Ready" : "Not Ready"));
              return stopped;
            }),
        // Intake sequence
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Starting intake at station");
                }),
            intake.intake(),
            switch (Constants.getRobot()) {
              case SIMBOT -> Commands.waitSeconds(0);
              case COMPBOT -> Commands.waitUntil(() -> !intake.getIntakeDistanceBool());
              default -> Commands.waitSeconds(0);
            },
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AutoStatus", "Stopping intake after collection");
                }),
            intake.stop(),
            RobotContainer.rotaryPart.coralScore().withTimeout(.01)));
  }

  public Command algaeAutoFreaktory(int targetTag) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotContainer.algaeMode = true;
              RobotContainer.intake.stop();
              Logger.recordOutput(
                  "AutoStatus", "Starting pathfinding to algae at tag " + targetTag);
              if (RobotContainer.autoPathingEnabled) {
                m_pathfinder.getPathfindingCommandAlgae(targetTag).schedule();
              }
            }),
        // Start preparatory movements during approach
        RobotContainer.rotaryPart.algaeGrab().withTimeout(.01),
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
                ? RobotContainer.elevator.toL2Algae().andThen(runOnce(() -> algaeHeight = 2))
                : RobotContainer.elevator.toL2Algae().andThen(runOnce(() -> algaeHeight = 3))),
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
                  holdAlgae = true;
                })));
  }

  public void processorAutoFreaktory() {}

  public void bargeAutoFreaktory() {}
}

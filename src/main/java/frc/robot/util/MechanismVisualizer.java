// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.RotaryPartSim;
import org.littletonrobotics.junction.Logger;

/**
 * This class handles visualization of the elevator and rotary arm mechanisms. It creates a 3D
 * component visualization for AdvantageScope.
 */
public class MechanismVisualizer extends SubsystemBase {
  // References to the actual subsystems
  private final RotaryPartSim m_rotaryPartSimSubsystem;

  // Height and movement parameters
  // Height and movement parameters - adjusted for continuous elevator
  private LoggedTunableNumber comp1MaxHeight =
      new LoggedTunableNumber(
          "Comp1MaxHeight", .925); // Adjust based on your actual elevator height

  private LoggedTunableNumber comp2HeightMultiplier =
      new LoggedTunableNumber(
          "Comp2HeightMultiplier", 0.5); // Adjust to represent the upper portion

  private LoggedTunableNumber comp3XOffset = new LoggedTunableNumber("Comp3XOffset", 0.28);
  private LoggedTunableNumber comp3ZOffset = new LoggedTunableNumber("Comp3ZOffset", 0.365);
  private LoggedTunableNumber comp3YOffset = new LoggedTunableNumber("Comp3YOffset", 0);
  private LoggedTunableNumber comp3YRotOffset = new LoggedTunableNumber("Comp3YRotOffset", -30);
  private LoggedTunableNumber armAngleMultiplier =
      new LoggedTunableNumber("ArmAngleMultiplier", -1);

  /**
   * Creates a new MechanismVisualizer.
   *
   * @param elevator The elevator subsystem
   * @param rotaryPart The rotary part subsystem
   */
  public MechanismVisualizer(Elevator elevator, RotaryPartSim rotaryPartSim) {
    m_rotaryPartSimSubsystem = rotaryPartSim;
  }

  @Override
  public void periodic() {
    try {
      // Only log at a reduced rate to avoid buffer overflow
      if (loggingCounter % 4 == 0) {
        logMechanismData();
      }
      loggingCounter++;

    } catch (Exception e) {
      // Log the error but don't crash the robot
      System.err.println("Error in MechanismVisualizer: " + e.getMessage());
    }
  }

  // Counter to reduce logging frequency
  private int loggingCounter = 0;

  private void logMechanismData() {
    try {
      // Update 3D Component Visualization
      double elevatorHeightMeters = RobotContainer.elevator.visualizationMeters;

      // For a continuous elevator, we want both components to share the same movement profile
      // Set the components to be proportional to the total height

      // First component - The base part of the elevator
      double totalMaxHeight = comp1MaxHeight.getAsDouble() + comp2HeightMultiplier.getAsDouble();
      double heightRatio = elevatorHeightMeters / totalMaxHeight;

      // Calculate component heights - they should move together proportionally
      double comp1Height =
          Math.min(heightRatio * comp1MaxHeight.getAsDouble(), comp1MaxHeight.getAsDouble());

      // Second component moves with the first component
      double comp2Height = comp1Height + (heightRatio * comp2HeightMultiplier.getAsDouble());

      // Calculate all the poses
      Pose3d component1Pose = new Pose3d(new Translation3d(0, 0, comp1Height), new Rotation3d());

      Pose3d component2Pose = new Pose3d(new Translation3d(0, 0, comp2Height), new Rotation3d());

      // Third component - The arm (with rotation)
      double armAngleDegrees =
          armAngleMultiplier.getAsDouble()
              * Units.radiansToDegrees(m_rotaryPartSimSubsystem.getAngleForVisualizationRads());

      Pose3d component3Pose =
          new Pose3d(
              new Translation3d(
                  comp3XOffset.getAsDouble(),
                  comp3YOffset.getAsDouble(),
                  comp2Height + comp3ZOffset.getAsDouble()),
              new Rotation3d(
                  0,
                  Units.degreesToRadians(armAngleDegrees)
                      + Units.degreesToRadians(comp3YRotOffset.getAsDouble()),
                  0));

      // Prepare arrays once - do all calculations before logging
      Pose3d[] componentPoses = new Pose3d[] {component1Pose, component2Pose, component3Pose};

      // Log the 3D component poses
      Logger.recordOutput("Mechanism3d/Components", componentPoses);
    } catch (Exception e) {
      System.err.println("Error in mechanism logging: " + e.getMessage());
    }
  }
}

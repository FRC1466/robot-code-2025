// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVisualizer {
  // Constants for visualization
  private static final double BASE_HEIGHT = 0.1; // Base height in meters

  // 2D visualization components
  private final String name;
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevatorMechanism;

  // 3D visualization origin
  private final Translation3d baseOrigin = new Translation3d(0.0, 0.0, 0.0);

  /**
   * Creates a new ElevatorVisualizer.
   *
   * @param name The name to use for logging
   */
  public ElevatorVisualizer(String name) {
    this.name = name;

    // Create 2D mechanism for visualization
    mechanism = new LoggedMechanism2d(1.0, 1.5, new Color8Bit(Color.kDarkGray));

    // Create the root and elevator components
    LoggedMechanismRoot2d root = mechanism.getRoot(name + " Root", 0.5, 0.1);

    // Create elevator visualization
    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator",
                BASE_HEIGHT, // Initial height
                90.0, // Straight up
                8.0, // Width
                new Color8Bit(Color.kFirstBlue)));
  }

  /**
   * Updates the elevator visualization with the current height.
   *
   * @param elevatorHeightMeters The current height of the elevator in meters
   */
  public void update(double elevatorHeightMeters) {
    // Ensure height is at least the minimum visualization height
    double displayHeight =
        Math.max(elevatorHeightMeters * Elevator.POSITION_CONVERSION_FACTOR, 0.05);

    // Update 2D mechanism
    elevatorMechanism.setLength(displayHeight);
    Logger.recordOutput("Mechanism2d/" + name, mechanism);

    // Calculate position of each stage based on current height
    double stage1Height, stage2Height;

    stage1Height = elevatorHeightMeters / 2 * Elevator.POSITION_CONVERSION_FACTOR;
    stage2Height = stage1Height + elevatorHeightMeters / 2 * Elevator.POSITION_CONVERSION_FACTOR;

    // Record 3D visualization data - base, stage 1, and stage 2
    Logger.recordOutput(
        "Mechanism3d/" + name + "/Elevator",
        new Pose3d(
            baseOrigin.plus(new Translation3d(0, 0, stage1Height)),
            new edu.wpi.first.math.geometry.Rotation3d()), // Stage 1
        new Pose3d(
            baseOrigin.plus(new Translation3d(0, 0, stage2Height)),
            new edu.wpi.first.math.geometry.Rotation3d()), // Stage 2
        new Pose3d(
            baseOrigin.plus(new Translation3d(0.28, 0.0, stage2Height + 0.365)),
            new edu.wpi.first.math.geometry.Rotation3d())); // Carriage
  }
}

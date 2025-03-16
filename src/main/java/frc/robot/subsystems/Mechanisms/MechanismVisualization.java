// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the elevator and arm mechanism in AdvantageScope. Compatible with the IO-layer
 * architecture.
 */
public class MechanismVisualization {
  // Constants for visualization - adjust these to match your robot dimensions
  private static final double ELEVATOR_WIDTH = 2; // meters
  private static final double ARM_LENGTH = 0.2; // meters
  private static final double MECHANISM_WIDTH = 1.5; // meters
  private static final double MECHANISM_HEIGHT = 1.5; // meters
  private static final double BASE_HEIGHT = 0.05; // meters

  // Mechanism objects
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d elevatorRoot;
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismLigament2d armMechanism;

  /** Creates a visualization of the elevator and rotary arm mechanism. */
  public MechanismVisualization() {
    // Create the overall mechanism container
    mechanism =
        new LoggedMechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT, new Color8Bit(Color.kDarkGray));

    // Create a root node for the elevator base at the bottom center
    elevatorRoot = mechanism.getRoot("Elevator Base", MECHANISM_WIDTH / 2, BASE_HEIGHT);

    // Create the elevator that extends upward
    elevatorMechanism =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                BASE_HEIGHT, // Initial height - will be updated
                90, // Straight up
                ELEVATOR_WIDTH,
                new Color8Bit(Color.kBlue)));

    // Create the arm that rotates at the top of the elevator
    armMechanism =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                "Arm",
                ARM_LENGTH,
                0, // Initial angle - will be updated
                ELEVATOR_WIDTH / 2,
                new Color8Bit(Color.kRed)));
  }

  /**
   * Updates the visualization with current heights and angles.
   *
   * @param elevatorHeightMeters Current height of the elevator in meters
   * @param armAngleRadians Current angle of the arm in radians
   */
  public void update(double elevatorHeightMeters, double armAngleRadians) {
    // Update elevator height - ensure minimum visualization height
    double displayHeight = Math.max(elevatorHeightMeters, 0.1);
    elevatorMechanism.setLength(displayHeight);

    // Update arm angle - convert from radians to degrees for display
    double armAngleDegrees = Math.toDegrees(armAngleRadians);
    armMechanism.setAngle(armAngleDegrees);

    // Log mechanism data using AdvantageKit
    Logger.recordOutput("Mechanism2d/RobotArm", mechanism);
  }

  /**
   * Gets the LoggedMechanism2d object for direct use if needed.
   *
   * @return The LoggedMechanism2d visualization
   */
  public LoggedMechanism2d getMechanism() {
    return mechanism;
  }
}

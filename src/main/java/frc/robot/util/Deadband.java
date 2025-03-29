// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

/**
 * Utility class for applying deadbands to joystick inputs with smooth transitions. Based on
 * research paper by Collin Ostrowski.
 *
 * <p>This implementation scales the output values so that: - Input values below deadband threshold
 * return 0 - Values above deadband smoothly transition without jumps - Full joystick range is
 * preserved (from deadband to 100%)
 */
public class Deadband {
  /**
   * Applies a deadband to a joystick value with smooth transition.
   *
   * @param value The raw joystick value (expected range: -1.0 to 1.0)
   * @param deadband The deadband threshold (positive, typically 0.05 to 0.1)
   * @return Processed value with deadband applied
   */
  public static double apply(double value, double deadband) {
    return apply(value, deadband, 1.0);
  }

  /**
   * Applies a deadband to a joystick value with smooth transition and maximum scaling.
   *
   * @param value The raw joystick value (expected range: -1.0 to 1.0)
   * @param deadband The deadband threshold (positive, typically 0.05 to 0.1)
   * @param maxOutput Maximum output magnitude (for scaling, typically 1.0)
   * @return Processed value with deadband applied and scaling
   */
  public static double apply(double value, double deadband, double maxOutput) {
    // Validate input parameters
    if (deadband < 0) {
      deadband = Math.abs(deadband);
    }

    if (deadband >= 1.0) {
      return 0.0; // Deadband is too large, all inputs would be zeroed
    }

    // Basic deadband check
    if (Math.abs(value) < deadband) {
      return 0.0;
    }

    // Apply scaling formula: (maxOutput / (1 - deadband)) * (value + (-sign(value) * deadband))
    double signValue = (value > 0) ? 1.0 : -1.0;
    return (maxOutput / (1.0 - deadband)) * (value + (-signValue * deadband));
  }

  /**
   * Applies deadband with custom maximum input value. Useful when joystick cannot reach its full
   * range.
   *
   * @param value The raw joystick value (expected range: -1.0 to 1.0)
   * @param deadband The deadband threshold (positive, typically 0.05 to 0.1)
   * @param maxOutput Maximum output magnitude (for scaling, typically 1.0)
   * @param maxInput The maximum input value the joystick can reach (e.g., 0.9)
   * @return Processed value with deadband applied and scaling
   */
  public static double apply(double value, double deadband, double maxOutput, double maxInput) {
    // Validate input parameters
    if (deadband < 0) {
      deadband = Math.abs(deadband);
    }

    if (deadband >= maxInput || maxInput <= 0) {
      return 0.0;
    }

    // Basic deadband check
    if (Math.abs(value) < deadband) {
      return 0.0;
    }

    // Apply scaling formula with custom maxInput
    double signValue = (value > 0) ? 1.0 : -1.0;
    return (maxOutput / (maxInput - deadband)) * (value + (-signValue * deadband));
  }
}

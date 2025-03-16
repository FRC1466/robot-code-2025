// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for elevator hardware IO operations. Based on AdvantageKit's IO pattern for hardware
 * abstraction.
 */
public interface ElevatorIO {
  /** The inputs class for the elevator subsystem. */
  @AutoLog
  public static class ElevatorIOInputs {
    /** Current position in rotations/meters */
    public double position = 0.0;

    /** Current velocity in rotations per second/meters per second */
    public double velocity = 0.0;

    /** Applied voltage to the elevator motors */
    public double appliedVolts = 0.0;

    /** Current being drawn by the motors in amps */
    public double currentAmpsMaster = 0.0;

    public double currentAmpsSlave = 0.0;

    /** Temperature of the motors in Celsius */
    public double[] tempCelsius = new double[] {0.0, 0.0};
  }

  /**
   * Updates the set of input values with the current I/O state. Called by the elevator subsystem
   * periodically.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the neutral mode of the elevator motors.
   *
   * @param isBrake True for brake mode, false for coast mode
   */
  public default void setNeutralMode(boolean isBrake) {}

  /**
   * Sets the elevator motor voltage.
   *
   * @param volts The voltage to set
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the elevator position sensor to a specific value.
   *
   * @param position The position to set in rotations/meters
   */
  public default void setPosition(double position) {}

  /**
   * Configure the PID constants on the motor controllers (if applicable).
   *
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The derivative gain
   */
  public default void configurePID(double kP, double kI, double kD) {}
}

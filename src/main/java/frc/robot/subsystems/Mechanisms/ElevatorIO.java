// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import org.littletonrobotics.junction.AutoLog;

/** Elevator hardware interface for all implementations (real and simulation) */
public interface ElevatorIO {
  /** Contains all the input data received from the elevator hardware */
  @AutoLog
  public static class ElevatorIOInputs {
    /** Current position in elevator encoder units */
    public double positionEncoderUnits = 0.0;

    /** Current velocity in encoder units per second */
    public double velocityEncoderUnitsPerSec = 0.0;

    /** Applied control output (voltage) */
    public double appliedVolts = 0.0;

    /** Current being drawn by the elevator motors in amps */
    public double currentAmps = 0.0;

    /** Whether the elevator has hit its lower limit */
    public boolean atLowerLimit = false;

    /** Whether the elevator has hit its upper limit */
    public boolean atUpperLimit = false;
  }

  /** Updates the inputs with the latest data from the hardware */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Set the neutral mode (brake/coast) */
  public default void setNeutralMode(boolean brake) {}

  /** Sets the elevator motor voltage */
  public default void setVoltage(double volts) {}

  /** Set the sensor position to the specified value */
  public default void setEncoderPosition(double positionEncoderUnits) {}
}

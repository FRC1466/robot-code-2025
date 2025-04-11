// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double followerAppliedVolts,
      double followerTorqueCurrentAmps,
      double followerSupplyCurrentAmps,
      double followerTempCelsius) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  /** Run elevator output shaft to positionRad with additional feedforward output */
  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}

  default void setEncoderPosition(double position) {}
}

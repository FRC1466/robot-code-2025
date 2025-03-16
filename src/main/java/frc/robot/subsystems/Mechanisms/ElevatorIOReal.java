// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.Constants;

/** Implementation of ElevatorIO for real robot hardware using TalonFX motors. */
public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX masterMotor;
  private final TalonFX leftSlaveFX;
  private double lastVoltage = 0.0;

  public ElevatorIOReal() {
    // Initialize TalonFX motor controllers
    masterMotor = new TalonFX(Constants.ElevatorConstants.masterID);
    leftSlaveFX = new TalonFX(Constants.ElevatorConstants.slaveID);

    // Configure default settings
    setNeutralMode(true); // Default to brake mode
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update position and velocity
    inputs.position = masterMotor.getPosition().getValueAsDouble();
    inputs.velocity = masterMotor.getVelocity().getValueAsDouble();

    // Update applied voltage
    inputs.appliedVolts = lastVoltage;

    // Update current draw
    inputs.currentAmpsMaster = masterMotor.getStatorCurrent().getValueAsDouble();
    inputs.currentAmpsSlave = leftSlaveFX.getStatorCurrent().getValueAsDouble();

    // Update temperature
    inputs.tempCelsius =
        new double[] {
          masterMotor.getDeviceTemp().getValueAsDouble(),
          leftSlaveFX.getDeviceTemp().getValueAsDouble()
        };
  }

  @Override
  public void setNeutralMode(boolean isBrake) {
    NeutralModeValue mode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    masterMotor.setNeutralMode(mode);
    leftSlaveFX.setNeutralMode(mode);
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setVoltage(volts);
    leftSlaveFX.setVoltage(-volts); // Note: One motor is inverted based on original code
    lastVoltage = volts;
  }

  @Override
  public void setPosition(double position) {
    masterMotor.setPosition(position);
    leftSlaveFX.setPosition(position);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Note: In the new architecture, PID control is handled in the subsystem,
    // not in the motor controllers. This method is here for potential direct
    // motor controller PID configuration if needed.
  }
}

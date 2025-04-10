// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.Constants;

/** IO implementation for elevator using TalonFX motors */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX masterMotor;
  private final TalonFX slaveMotor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ElevatorIOTalonFX() {
    // Configure real motors
    masterMotor = new TalonFX(Constants.ElevatorConstants.masterID);
    slaveMotor = new TalonFX(Constants.ElevatorConstants.slaveID);

    // Initialize motors to safe state
    masterMotor.setVoltage(0);
    slaveMotor.setVoltage(0);

    // Set brake mode for safety
    setNeutralMode(true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionEncoderUnits = masterMotor.getPosition().getValueAsDouble();
    inputs.velocityEncoderUnitsPerSec = masterMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = masterMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = masterMotor.getSupplyCurrent().getValueAsDouble();

    // The TalonFX doesn't have built-in limit switch support in this configuration,
    // so we'll assume no limit switches for now
    inputs.atLowerLimit = false;
    inputs.atUpperLimit = false;
  }

  @Override
  public void setNeutralMode(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    masterMotor.setNeutralMode(mode);
    slaveMotor.setNeutralMode(mode);
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setControl(voltageRequest.withOutput(volts));
    slaveMotor.setControl(voltageRequest.withOutput(-volts)); // Inverted for slave motor
  }

  @Override
  public void setEncoderPosition(double positionEncoderUnits) {
    masterMotor.setPosition(positionEncoderUnits);
    slaveMotor.setPosition(-positionEncoderUnits); // Inverted for slave motor
  }
}

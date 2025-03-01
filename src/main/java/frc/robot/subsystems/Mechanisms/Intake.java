// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;

  @SuppressWarnings("unused")
  private double prevMotorPose = 0;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);

  @SuppressWarnings("unused")
  private double filteredCurrent;

  private boolean highCurrentBool;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /** Create a new Gripper subsystem. */
  public Intake() {
    intakeMotor = new TalonFX(15);
  }

  public void setVoltage(double outputVoltage) {
    intakeMotor.setVoltage(outputVoltage);
  }

  public Command stop() {
    return runOnce(() -> setVoltage(0));
  }

  public Command reverseIntake() {
    return runOnce(() -> setVoltage(4));
  }

  public Command algaeHold() {
    return runOnce(() -> setVoltage(.75));
  }

  public Command coralHold() {
    return runOnce(() -> setVoltage(.2));
  }

  public Command intake() {
    return runOnce(() -> setVoltage(-4));
  }

  public Command outTake() {
    return runOnce(() -> setVoltage(-6));
  }

  public boolean getIntakeDistanceBool() {
    return (m_colorSensor.getProximity() <= 120);
  }

  public double getCurrent(TalonFX intakeMotor) {
    return (intakeMotor.getSupplyCurrent()).getValueAsDouble();
  }

  public boolean getHighCurrent() {
    if (intakeMotor.getTorqueCurrent().getValueAsDouble() > 10) {
      highCurrentBool = true;
    } else if (intakeMotor.getTorqueCurrent().getValueAsDouble() < 3) {
      highCurrentBool = false;
    }
    return highCurrentBool;
  }

  public void periodic() {
    filteredCurrent = currentFilter.calculate(getCurrent(intakeMotor));
    // SmartDashboard.putNumber("ColorSensed", m_colorSensor.getProximity());
    // SmartDashboard.putBoolean("ColorSensed boolean", (m_colorSensor.getProximity() <= 120));
    // Logger.recordOutput(
    //     "Angle Change", (intakeMotor.getPosition().getValueAsDouble() - prevMotorPose));
    prevMotorPose = intakeMotor.getPosition().getValueAsDouble();
    // Logger.recordOutput("motor current", intakeMotor.getTorqueCurrent().getValueAsDouble());
  }
}

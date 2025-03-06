// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final SparkMax funnelMotor;

  private boolean highCurrentBool;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /** Create a new Gripper subsystem. */
  public Intake() {
    intakeMotor = new TalonFX(15);
    funnelMotor = new SparkMax(18, SparkMax.MotorType.kBrushless);
  }

  public void setIntakeVoltage(double outputVoltage) {
    intakeMotor.setVoltage(outputVoltage);
  }

  public void setFunnelVoltage(double outputVoltage) {
    funnelMotor.setVoltage(outputVoltage);
  }

  public Command stop() {
    return runOnce(
        () -> {
          setIntakeVoltage(0);
          setFunnelVoltage(0);
        });
  }

  public Command reverseIntake() {
    return runOnce(
        () -> {
          setIntakeVoltage(6);
          setFunnelVoltage(0);
        });
  }

  public Command algaeHold() {
    return runOnce(
        () -> {
          setIntakeVoltage(.4);
          setFunnelVoltage(0);
        });
  }

  public Command coralHold() {
    return runOnce(
        () -> {
          setIntakeVoltage(.3);
          setFunnelVoltage(0);
        });
  }

  public Command intake() {
    return runOnce(
        () -> {
          setIntakeVoltage(-2);
          setFunnelVoltage(-1.5);
        });
  }

  public Command outTake() {
    return runOnce(
        () -> {
          setIntakeVoltage(-6);
          setFunnelVoltage(0);
        });
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
    Logger.recordOutput("ColorSensed boolean", (m_colorSensor.getProximity() <= 120));
    Logger.recordOutput("Intake Motor Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput("Intake Motor High Current", highCurrentBool);
  }
}

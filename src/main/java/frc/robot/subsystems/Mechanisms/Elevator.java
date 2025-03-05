// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final TalonFX masterMotor, leftSlaveFX;

  @SuppressWarnings("unused")
  private static Elevator instance;

  private PIDController elevatorPID;
  private double localSetpoint = 0;
  private DoubleSupplier overrideFeedforward = () -> 0;

  private double peakOutput;

  public Elevator() {
    masterMotor = new TalonFX(Constants.ElevatorConstants.masterID);

    leftSlaveFX = new TalonFX(Constants.ElevatorConstants.slaveID);

    peakOutput = Constants.ElevatorConstants.elevatorPosition.peakOutput;
    elevatorPID =
        new PIDController(
            Constants.ElevatorConstants.elevatorPosition.P,
            Constants.ElevatorConstants.elevatorPosition.I,
            Constants.ElevatorConstants.elevatorPosition.D);
    elevatorPID.setTolerance(.1);
    setGoal(1);
    masterMotor.setVoltage(0);
    leftSlaveFX.setVoltage(0);
    setNeutralMode(NeutralModeValue.Brake);
  }

  private void setNeutralMode(NeutralModeValue neutralMode) {
    masterMotor.setNeutralMode(neutralMode);
    leftSlaveFX.setNeutralMode(neutralMode);
  }

  public void setSelectedSensorPosition(double position) {
    masterMotor.setPosition(position);
    leftSlaveFX.setPosition(position);
  }

  public void setP(double p) {
    elevatorPID.setP(p);
  }

  public void setPeakOutput(double peak) {
    peakOutput = peak;
  }

  public double getElevatorHeight() {
    double height = masterMotor.getPosition().getValueAsDouble();
    return height;
  }

  public void goToGoal(double goal) {
    localSetpoint = goal;
    elevatorPID.setSetpoint(goal);
    SmartDashboard.putNumber("Elevator PID Setpoint", goal);
  }

  public void setMotor(double percent) {
    masterMotor.set(percent);
    leftSlaveFX.set(-percent);
  }

  public Command setGoal(double goal) {
    return runOnce(() -> goToGoal(goal));
  }

  // lower this and lower PID
  public Command toBottom() {
    return runOnce(() -> goToGoal(.5));
  }

  public Command toL1() {
    return runOnce(() -> goToGoal(10));
  }

  public Command toL2() {
    return runOnce(() -> goToGoal(15));
  }

  public Command toL2Algae() {
    return runOnce(() -> goToGoal(25));
  }

  public Command toL3() {
    return runOnce(() -> goToGoal(30));
  }

  public Command toL3Algae() {
    return runOnce(() -> goToGoal(42));
  }

  public Command toL4() {
    return runOnce(() -> goToGoal(63));
  }

  public Command toL4Algae() {
    return runOnce(() -> goToGoal(66));
  }

  public Command toStation() {
    return runOnce(() -> goToGoal(10));
  }

  public Command toProcessor() {
    return runOnce(() -> goToGoal(3.5));
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            elevatorPID.calculate(getElevatorHeight(), localSetpoint), -peakOutput, peakOutput);
    var feedforward = getElevatorHeight() * Constants.ElevatorConstants.elevatorPosition.F;
    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());
    SmartDashboard.putNumber("Elevator PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Elevator Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setFeedforward(DoubleSupplier feedforward) {
    overrideFeedforward = feedforward;
  }

  public void setMotorVoltage(double volts) {
    masterMotor.setVoltage(volts);
    leftSlaveFX.setVoltage(-volts);
  }

  public Command setElevatorVoltage(double volts) {
    return runOnce(() -> setMotorVoltage(volts));
  }

  public void reset() {
    elevatorPID.reset();
  }

  @Override
  public void periodic() {
    setArmHold();

    Logger.recordOutput("Elevator Position", getElevatorHeight());
    Logger.recordOutput("Elevator Desired Positon", elevatorPID.getSetpoint());
  }
}

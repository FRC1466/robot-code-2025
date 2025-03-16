// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  // IO management
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Control objects
  public final PIDController elevatorPID;
  private double localSetpoint = 0;
  private DoubleSupplier overrideFeedforward = () -> 0;
  private double peakOutput;

  /**
   * Creates a new Elevator subsystem.
   *
   * @param io The IO interface implementation to use (real or simulation)
   */
  public Elevator(ElevatorIO io) {
    this.io = io;

    peakOutput = Constants.ElevatorConstants.elevatorPosition.peakOutput;
    elevatorPID =
        new PIDController(
            Constants.ElevatorConstants.elevatorPosition.P,
            Constants.ElevatorConstants.elevatorPosition.I,
            Constants.ElevatorConstants.elevatorPosition.D);
    elevatorPID.setTolerance(.1);

    setGoal(1);
    io.setVoltage(0);
    setNeutralMode(true); // Set to brake mode
  }

  private void setNeutralMode(boolean isBrake) {
    io.setNeutralMode(isBrake);
  }

  public void setSelectedSensorPosition(double position) {
    io.setPosition(position);
  }

  public void setP(double p) {
    elevatorPID.setP(p);
  }

  public void setPeakOutput(double peak) {
    peakOutput = peak;
  }

  public double getElevatorHeight() {
    return inputs.position;
  }

  public void goToGoal(double goal) {
    localSetpoint = goal;
    elevatorPID.setSetpoint(goal);
    SmartDashboard.putNumber("Elevator PID Setpoint", goal);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
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
    return runOnce(() -> goToGoal(14));
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
            elevatorPID.calculate(inputs.position, localSetpoint), -peakOutput, peakOutput);
    var feedforward = inputs.position * Constants.ElevatorConstants.elevatorPosition.F;
    setVoltage(motorOutput + feedforward + overrideFeedforward.getAsDouble());
    SmartDashboard.putNumber("Elevator PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Elevator Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setFeedforward(DoubleSupplier feedforward) {
    overrideFeedforward = feedforward;
  }

  public void setMotorVoltage(double volts) {
    io.setVoltage(volts);
  }

  public Command setElevatorVoltage(double volts) {
    return runOnce(() -> setMotorVoltage(volts));
  }

  public void reset() {
    elevatorPID.reset();
  }

  @Override
  public void periodic() {
    // Update IO inputs and log them
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Apply control logic
    setArmHold();

    // Log additional data
    Logger.recordOutput("Elevator/Position", getElevatorHeight());
    Logger.recordOutput("Elevator/DesiredPosition", elevatorPID.getSetpoint());
    Logger.recordOutput("Elevator/Velocity", inputs.velocity);
    Logger.recordOutput("Elevator/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Elevator/CurrentAmpsMaster", inputs.currentAmpsMaster);
    Logger.recordOutput("Elevator/CurrentAmpsSlave", inputs.currentAmpsSlave);
    Logger.recordOutput("Elevator/TemperatureCelsius", inputs.tempCelsius);
  }
}

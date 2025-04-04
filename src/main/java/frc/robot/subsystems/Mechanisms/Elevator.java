// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final TalonFX masterMotor, leftSlaveFX;

  private PIDController elevatorPID;
  private double localSetpoint = 0;
  private DoubleSupplier overrideFeedforward = () -> 0;

  @SuppressWarnings("unused")
  private ElevatorFeedforward m_feedforward;

  private SysIdRoutine m_sysIdRoutine;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private double peakOutput;
  public double visualizationMeters = 0.0;

  public Elevator() {
    // Configure real motors
    masterMotor = new TalonFX(Constants.ElevatorConstants.masterID);
    leftSlaveFX = new TalonFX(Constants.ElevatorConstants.slaveID);

    peakOutput = Constants.ElevatorConstants.elevatorPosition.peakOutput;
    elevatorPID =
        new PIDController(
            Constants.ElevatorConstants.elevatorPosition.P,
            Constants.ElevatorConstants.elevatorPosition.I,
            Constants.ElevatorConstants.elevatorPosition.D);
    elevatorPID.setTolerance(.1);

    // TODO: UNDO THIS ONCE DONE TESTING
    // setGoal(.1);
    masterMotor.setVoltage(0);
    leftSlaveFX.setVoltage(0);
    // TODO: Remove this once it is completed
    setNeutralMode(NeutralModeValue.Brake);
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(.5, Volts.per(Seconds)),
                Volts.of(2),
                Seconds.of(10),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> masterMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this));
  }

  private void setNeutralMode(NeutralModeValue neutralMode) {
    masterMotor.setNeutralMode(neutralMode);
    leftSlaveFX.setNeutralMode(neutralMode);
  }

  public void setSelectedSensorPosition(double position) {
    masterMotor.setPosition(position);
    leftSlaveFX.setPosition(position);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setP(double p) {
    elevatorPID.setP(p);
  }

  public void setPeakOutput(double peak) {
    peakOutput = peak;
  }

  public double getElevatorHeight() {
    return masterMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorHeightMeters() {
    return getElevatorHeight() * 0.02205522;
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

  public boolean atSetpoint() {
    return elevatorPID.atSetpoint();
  }

  public Command setGoal(double goal) {
    return runOnce(() -> goToGoal(goal));
  }

  // Command methods for different elevator positions
  public Command toBottom() {
    return runOnce(() -> goToGoal(.1));
  }

  public Command toL1() {
    return runOnce(() -> goToGoal(10));
  }

  public Command toL2() {
    return runOnce(() -> goToGoal(14));
  }

  public Command toL2Algae() {
    return runOnce(() -> goToGoal(24));
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
    return runOnce(() -> goToGoal(5.5));
  }

  public Command toTestingHeight() {
    return runOnce(() -> goToGoal(8));
  }

  public void setArmHold() {
    // TODO: UNCOMMENT!!! THIS IS IMPORTANT!!!
    var motorOutput =
        MathUtil.clamp(
            elevatorPID.calculate(getElevatorHeight(), localSetpoint), -peakOutput, peakOutput);
    var feedforward = getElevatorHeight() * Constants.ElevatorConstants.elevatorPosition.F;
    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());

    Logger.recordOutput("Elevator PID Output", motorOutput);
    Logger.recordOutput("Elevator Feedforward", feedforward);
    Logger.recordOutput("Elevator Feedforward Override", overrideFeedforward.getAsDouble());
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
    Logger.recordOutput("Elevator Desired Position", elevatorPID.getSetpoint());
  }
}

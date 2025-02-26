// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Adjust constants for better visualization
  private static final double SIMULATION_SPEED = 2; // Increased from 0.02
  private static final double VISUALIZATION_SCALE = 0.1; // Meters per tick
  private static final double ELEVATOR_MIN_LENGTH = 0.3; // Starting height
  private static final double ELEVATOR_MAX_LENGTH = 1.5; // Max height

  private final TalonFX masterMotor, leftSlaveFX;

  @SuppressWarnings("unused")
  private static Elevator instance;

  private PIDController elevatorPID;
  private double localSetpoint = 0;
  private DoubleSupplier overrideFeedforward = () -> 0;

  // Will use when we install hall sensor
  // DigitalInput hallBottom = new DigitalInput(0);

  // figure out what this is
  @SuppressWarnings("unused")
  private static final double TICKS_PER_INCH = 1;

  // Also figure this out
  @SuppressWarnings("unused")
  private static final double HOME_POSITION_INCHES = 0;

  @SuppressWarnings("unused")
  private static final double MAX_POSITION_TICKS = 64;

  private double peakOutput;

  // private final LoggedMechanism2d robotMech =
  //    new LoggedMechanism2d(1.0, 1.0); // 1x1 meter visualization

  // @AutoLogOutput private final LoggedMechanismRoot2d elevatorRoot;
  // @AutoLogOutput private final LoggedMechanismLigament2d elevatorLigament;
  // @AutoLogOutput private final LoggedMechanismLigament2d wristLigament;

  // private final RotatyPart wrist;

  private double simulatedPosition = 0.0;
  private boolean isSimulation;

  public Elevator(RotatyPart wrist) {
    isSimulation = RobotBase.isSimulation();

    masterMotor = new TalonFX(Constants.ElevatorConstants.masterID);

    leftSlaveFX = new TalonFX(Constants.ElevatorConstants.slaveID);

    peakOutput = Constants.ElevatorConstants.elevatorPosition.peakOutput;
    elevatorPID =
        new PIDController(
            Constants.ElevatorConstants.elevatorPosition.P,
            Constants.ElevatorConstants.elevatorPosition.I,
            Constants.ElevatorConstants.elevatorPosition.D);
    elevatorPID.setTolerance(.1);
    setGoal(.5);
    masterMotor.setVoltage(0);
    leftSlaveFX.setVoltage(0);
    setNeutralMode(NeutralModeValue.Brake);

    // this.wrist = wrist;

    // Initialize mechanism visualization
    // elevatorRoot = robotMech.getRoot("Elevator", 0.6, 0);

    // elevatorLigament =
    //    elevatorRoot.append(
    //        new LoggedMechanismLigament2d(
    //            "ElevatorStage",
    //            ELEVATOR_MIN_LENGTH,
    //            90, // vertical
    //            6, // width
    //            new Color8Bit(Color.kBlue)));

    // wristLigament =
    //    elevatorLigament.append(
    //        new LoggedMechanismLigament2d(
    //            "Wrist",
    //            0.2, // 20cm wrist length
    //            0, // horizontal to start
    //            4, // smaller width
    //            new Color8Bit(Color.kRed)));

    // SmartDashboard.putData("Robot Mechanism", robotMech);
  }

  private void setNeutralMode(NeutralModeValue neutralMode) {
    masterMotor.setNeutralMode(neutralMode);
    leftSlaveFX.setNeutralMode(neutralMode);
  }

  public void setSelectedSensorPosition(double position) {
    if (!isSimulation) {
      masterMotor.setPosition(position);
      leftSlaveFX.setPosition(position);
    }
    simulatedPosition = position;
  }

  public void setP(double p) {
    elevatorPID.setP(p);
  }

  public void setPeakOutput(double peak) {
    peakOutput = peak;
  }

  public double getElevatorHeight() {
    if (!isSimulation) {
      return masterMotor.getPosition().getValueAsDouble();
    }
    return simulatedPosition;
  }

  public void goToGoal(double goal) {
    localSetpoint = goal;
    elevatorPID.setSetpoint(goal);
    SmartDashboard.putNumber("Elevator PID Setpoint", goal);
  }

  public void setMotor(double percent) {
    if (!isSimulation) {
      masterMotor.set(percent);
      leftSlaveFX.set(-percent);
    } else {
      // Faster simulation physics
      simulatedPosition += percent * SIMULATION_SPEED;
      simulatedPosition = MathUtil.clamp(simulatedPosition, 0, MAX_POSITION_TICKS);
    }
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
    return runOnce(() -> goToGoal(32));
  }

  public Command toL4() {
    return runOnce(() -> goToGoal(60));
  }

  public Command toStation() {
    return runOnce(() -> goToGoal(10));
  }

  public Command removeAlgaeLow() {
    return runOnce(() -> goToGoal(10));
  }

  public Command removeAlgaeHigh() {
    return runOnce(() -> goToGoal(10));
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

    // Scale height for visualization
    double heightMeters = getElevatorHeight() * VISUALIZATION_SCALE;
    double visualHeight =
        MathUtil.clamp(
            ELEVATOR_MIN_LENGTH + heightMeters, ELEVATOR_MIN_LENGTH, ELEVATOR_MAX_LENGTH);

    // Update visualization
    // elevatorLigament.setLength(visualHeight);

    // Update wrist visualization
    // wristLigament.setAngle(wrist.getPosition().getDegrees());

    // Debug values
    SmartDashboard.putBoolean("Is Simulation", isSimulation);
    SmartDashboard.putNumber("Raw Height", getElevatorHeight());
    SmartDashboard.putNumber("Visual Height (m)", visualHeight);

    // Update visualization and logging
    SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
    SmartDashboard.putNumber("Get Elevator P", elevatorPID.getP());
    SmartDashboard.putNumber("Get Elevator PeakOutput", peakOutput);
    SmartDashboard.putNumber("Elevator Desired Position", elevatorPID.getSetpoint());
    SmartDashboard.putNumber("Elevator Error", elevatorPID.getError());

    Logger.recordOutput("Elevator Position", getElevatorHeight());

    // Update mechanism visualization
    heightMeters = (getElevatorHeight() / TICKS_PER_INCH) * 0.0254;
    // elevatorLigament.setLength(ELEVATOR_MIN_LENGTH + heightMeters);
  }
}

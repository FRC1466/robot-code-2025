// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Motion profile constraints
  private static final double MAX_VELOCITY_TICKS_PER_SEC = 150;
  private static final double MAX_ACCELERATION_TICKS_PER_SEC_SQUARED = 75;

  // Profiled PID controller
  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          MAX_VELOCITY_TICKS_PER_SEC, MAX_ACCELERATION_TICKS_PER_SEC_SQUARED);
  private final ProfiledPIDController profiledPIDController;

  // Profile state tracking
  @SuppressWarnings("unused")
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

  @SuppressWarnings("unused")
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  // Tunable PID and motion profile parameters
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 1.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 1.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/MaxVelocity", MAX_VELOCITY_TICKS_PER_SEC);
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/MaxAcceleration", MAX_ACCELERATION_TICKS_PER_SEC_SQUARED);

  // IO and inputs
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private DoubleSupplier overrideFeedforward = () -> 0;

  private ElevatorFeedforward m_feedforward;

  private SysIdRoutine m_sysIdRoutine;

  public double visualizationMeters = 0.0;

  private static ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer("Measured");
  private static ElevatorVisualizer setPointVisualizer = new ElevatorVisualizer("Setpoint");

  // Position conversion factor - motor rotations to meters
  public static final double POSITION_CONVERSION_FACTOR = 0.02205522;
  public static final double drumRadius = Units.inchesToMeters(1.0);

  public Elevator(ElevatorIO io) {
    this.io = io;

    // Setup feedforward controller for gravity compensation
    m_feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get());

    switch (Constants.getRobot()) {
      case COMPBOT -> {
        kP.initDefault(1500);
        kD.initDefault(30);
      }
      case SIMBOT -> {
        kP.initDefault(5000);
        kD.initDefault(2000);
      }
      default -> Commands.none();
    }

    // Initialize profiled PID controller with default parameters
    profiledPIDController = new ProfiledPIDController(kP.get(), kI.get(), kD.get(), constraints);
    profiledPIDController.setTolerance(0.02); // 2cm position tolerance

    // Set brake mode for safety
    io.setBrakeMode(true);

    // Configure system identification routine
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(.5, Volts.per(Seconds)),
                Volts.of(2),
                Seconds.of(10),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.runVolts(volts.in(Volts)), null, this));
  }

  public void setSelectedSensorPosition(double position) {
    io.setEncoderPosition(position);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setP(double p) {
    profiledPIDController.setP(p);
  }

  public void setI(double i) {
    profiledPIDController.setI(i);
  }

  public void setD(double d) {
    profiledPIDController.setD(d);
  }

  /**
   * @return Current position in motor encoder units
   */
  public double getElevatorHeight() {
    return inputs.data.positionRad();
  }

  /**
   * @return Current position in meters
   */
  public double getPositionMeters() {
    return (inputs.data.positionRad()) * 0.02205522;
  }

  /**
   * Sets the goal position for the elevator with motion profiling
   *
   * @param goal The target position in encoder units
   */
  public void goToGoal(double goal) {
    goalState = new TrapezoidProfile.State(goal, 0);

    // Reset the profiled PID controller with the current state
    double currentPosition = getElevatorHeight();
    currentState = new TrapezoidProfile.State(currentPosition, getCurrentVelocity());
    profiledPIDController.reset(currentPosition);

    // Set the goal for the profiled controller
    profiledPIDController.setGoal(goal);
  }

  /**
   * @return Current velocity in motor units per second
   */
  public double getCurrentVelocity() {
    return inputs.data.velocityRadPerSec();
  }

  /**
   * @return Whether the elevator has reached the target position
   */
  public boolean atSetpoint() {
    return profiledPIDController.atGoal();
  }

  // Command methods for different elevator positions
  public Command toBottom() {
    return runOnce(() -> goToGoal(0.1));
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

  /**
   * Sets a custom feedforward provider for the elevator
   *
   * @param feedforward DoubleSupplier that provides feedforward values
   */
  public void setFeedforward(DoubleSupplier feedforward) {
    overrideFeedforward = feedforward;
  }

  /**
   * Sets motor voltage directly
   *
   * @param volts Voltage to apply
   */
  public void setMotorVoltage(double volts) {
    io.runVolts(volts);
  }

  /**
   * Command to set a specific voltage
   *
   * @param volts Voltage to apply
   * @return Command that sets voltage
   */
  public Command setElevatorVoltage(double volts) {
    return runOnce(() -> setMotorVoltage(volts));
  }

  /** Resets the PID controller and motion profile */
  public void reset() {
    double currentPosition = getElevatorHeight();
    profiledPIDController.reset(new TrapezoidProfile.State(currentPosition, getCurrentVelocity()));
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update PID and motion profile parameters if changed
    if (kP.hasChanged(0) || kI.hasChanged(0) || kD.hasChanged(0)) {
      profiledPIDController.setPID(kP.get(), kI.get(), kD.get());
    }
    if (kS.hasChanged(0) || kV.hasChanged(0) || kG.hasChanged(0)) {
      m_feedforward.setKs(kS.get());
      m_feedforward.setKv(kV.get());
      m_feedforward.setKg(kG.get());
    }

    if (maxVelocity.hasChanged(0) || maxAcceleration.hasChanged(0)) {
      TrapezoidProfile.Constraints newConstraints =
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
      profiledPIDController.setConstraints(newConstraints);
    }

    // Get the current measurement
    double currentPosition = getElevatorHeight();
    double currentPositionMeters = getPositionMeters();

    // Calculate the next output using the profiled PID controller
    double pidOutput = profiledPIDController.calculate(currentPosition);

    // Calculate feedforward (gravity compensation)
    double feedforwardOutput = overrideFeedforward.getAsDouble();

    // Combine PID output and feedforward
    double combinedOutput = pidOutput + feedforwardOutput;

    // Apply the control output to the elevator motor with voltage limiting
    combinedOutput = MathUtil.clamp(combinedOutput, -12.0, 12.0);
    io.runVolts(combinedOutput);

    // Log debugging information
    Logger.recordOutput("Elevator/CurrentPosition", currentPosition);
    Logger.recordOutput("Elevator/CurrentPositionMeters", currentPositionMeters);
    Logger.recordOutput("Elevator/ProfileSetpoint", profiledPIDController.getSetpoint().position);
    Logger.recordOutput("Elevator/ProfileVelocity", profiledPIDController.getSetpoint().velocity);
    Logger.recordOutput("Elevator/PIDOutput", pidOutput);
    Logger.recordOutput("Elevator/FeedforwardOutput", feedforwardOutput);
    Logger.recordOutput("Elevator/CombinedOutput", combinedOutput);
    Logger.recordOutput("Elevator/AtGoal", profiledPIDController.atGoal());
    Logger.recordOutput("Elevator/PositionError", profiledPIDController.getPositionError());
    elevatorVisualizer.update(currentPosition);
    setPointVisualizer.update(profiledPIDController.getSetpoint().position);

    LoggedTracer.record("Elevator");
  }
}

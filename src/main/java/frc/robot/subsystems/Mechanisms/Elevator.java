// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotType;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  private final TalonFX masterMotor, leftSlaveFX;

  private PIDController elevatorPID;
  private double localSetpoint = 0;
  private DoubleSupplier overrideFeedforward = () -> 0;

  // Simulation classes
  private DCMotor m_elevatorGearboxSim;
  private Encoder m_encoder;
  private EncoderSim m_encoderSim;
  private ElevatorSim m_elevatorSim;
  private ElevatorFeedforward m_feedforward;

  private double peakOutput;

  // Create a Mechanism2d visualization of the elevator
  private final LoggedMechanism2d m_Mechanism2d = new LoggedMechanism2d(20, 200);
  private final LoggedMechanismRoot2d m_ElevatorSimMechRoot2d =
      m_Mechanism2d.getRoot("Elevator Root", 10, 0);
  private LoggedMechanismLigament2d m_elevatorMech2d;

  // Threshold values for determining which visualization to use
  private final double RESTING_THRESHOLD = 2.0; // 0-2 inches
  private final double INTAKE_THRESHOLD = 12.0; // 3-12 inches
  private final double L2_THRESHOLD = 20.0; // 13-20 inches
  private final double L2_ALGAE_THRESHOLD = 28.0; // 21-28 inches
  private final double L3_THRESHOLD = 36.0; // 29-36 inches
  private final double L3_ALGAE_THRESHOLD = 50.0; // 37-50 inches
  private final double L4_THRESHOLD = 64.0; // 51-64 inches
  public double visualizationMeters = 0.0;

  // 65+ is barge

  private void createSimMotors() {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT:
        break;
      case SIMBOT:
        // Initialize simulation components
        m_elevatorGearboxSim = DCMotor.getKrakenX60(2);
        m_encoder = new Encoder(29, 30);
        m_encoderSim = new EncoderSim(m_encoder);

        // Configure encoder for simulation
        m_encoder.setDistancePerPulse(2.0 * Math.PI * Units.inchesToMeters(2) / 4096);

        // Create elevator feedforward
        m_feedforward =
            new ElevatorFeedforward(
                0.0, // kS - static friction
                0.78, // kG - gravity compensation
                1.0, // kV - velocity
                0.0 // kA - acceleration
                );
        break;
      default:
        break;
    }
  }

  public Elevator() {
    // Create NetworkTable entry for the Pose2d (for visualization)
    final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    final NetworkTable table = ntInstance.getTable("Elevator");

    switch (Constants.getRobot()) {
      case COMPBOT:
        break;
      case SIMBOT:
        createSimMotors();
        // Create elevator simulation with proper parameters
        m_elevatorSim =
            new ElevatorSim(
                m_elevatorGearboxSim,
                10, // gearing
                2, // carriage mass (kg)
                Units.inchesToMeters(2), // drum radius
                0.0, // min height
                5, // max height
                true, // simulate gravity
                0); // initial position
        break;
      default:
        break;
    }

    // Now that m_elevatorSim exists, we can initialize the ligament
    if (Constants.getRobot() == RobotType.SIMBOT) {
      m_elevatorMech2d =
          m_ElevatorSimMechRoot2d.append(new LoggedMechanismLigament2d("Elevator", 0, 90));
    } else {
      // Default visualization with zero length when not in simulation
      m_elevatorMech2d =
          m_ElevatorSimMechRoot2d.append(new LoggedMechanismLigament2d("Elevator", 0, 90));
    }

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
    setGoal(.1);
    masterMotor.setVoltage(0);
    leftSlaveFX.setVoltage(0);
    setNeutralMode(NeutralModeValue.Brake);

    // Publish the mechanism visualization
    Logger.recordOutput("Elevator Mech", m_Mechanism2d);
  }

  public void updateMechanism() {
    switch (Constants.getRobot()) {
      case COMPBOT:
        // Multiplying from bradys to meters
        m_elevatorMech2d.setLength((masterMotor.getPosition().getValueAsDouble()) * 0.02205522);

        break;
      case SIMBOT:
        // Get current setpoint
        double currentSetpoint = masterMotor.getPosition().getValueAsDouble();

        // Use a direct multiplier instead of threshold-based visualization
        visualizationMeters = currentSetpoint * 0.02205522;

        // Still log position type for debugging purposes
        if (currentSetpoint <= RESTING_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "Resting");
        } else if (currentSetpoint <= INTAKE_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "Intake/L1");
        } else if (currentSetpoint <= L2_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "L2");
        } else if (currentSetpoint <= L2_ALGAE_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "L2 Algae");
        } else if (currentSetpoint <= L3_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "L3");
        } else if (currentSetpoint <= L3_ALGAE_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "L3 Algae");
        } else if (currentSetpoint <= L4_THRESHOLD) {
          Logger.recordOutput("Elevator/VisualizedPosition", "L4");
        } else {
          Logger.recordOutput("Elevator/VisualizedPosition", "Barge/L4 Algae");
        }

        // Set the ligament length to the direct calculation
        m_elevatorMech2d.setLength(visualizationMeters);

        // Log both the real setpoint and visualization position for debugging
        Logger.recordOutput("Elevator/SetpointInches", currentSetpoint);
        Logger.recordOutput("Elevator/VisualizationMeters", visualizationMeters);
        break;
      default:
        break;
    }
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
    if (Constants.getRobot() == RobotType.SIMBOT) {
      return Units.metersToInches(m_elevatorSim.getPositionMeters());
    } else {
      return masterMotor.getPosition().getValueAsDouble();
    }
  }

  public void goToGoal(double goal) {
    localSetpoint = goal;
    elevatorPID.setSetpoint(goal);
    SmartDashboard.putNumber("Elevator PID Setpoint", goal);
  }

  public void setMotor(double percent) {
    if (Constants.getRobot() == RobotType.SIMBOT) {
      // For simulation, we'll apply the voltage directly in simulationPeriodic
    } else {
      masterMotor.set(percent);
      leftSlaveFX.set(-percent);
    }
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

  public Command toL2FreakyAlgae() {
    return runOnce(() -> goToGoal(34));
  }

  public Command toL3() {
    return runOnce(() -> goToGoal(30));
  }

  public Command toL3Algae() {
    return runOnce(() -> goToGoal(42));
  }

  public Command toL3FreakyAlgae() {
    return runOnce(() -> goToGoal(52));
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

  public Command toTestingHeight() {
    return runOnce(() -> goToGoal(8));
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            elevatorPID.calculate(getElevatorHeight(), localSetpoint), -peakOutput, peakOutput);
    var feedforward = getElevatorHeight() * Constants.ElevatorConstants.elevatorPosition.F;

    if (Constants.getRobot() == RobotType.SIMBOT) {
      // Store the calculated output to be used in simulationPeriodic
      double voltage =
          (motorOutput + feedforward + overrideFeedforward.getAsDouble())
              * RobotController.getBatteryVoltage();
      m_elevatorSim.setInput(voltage);
    } else {
      setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());
    }

    SmartDashboard.putNumber("Elevator PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Elevator Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setFeedforward(DoubleSupplier feedforward) {
    overrideFeedforward = feedforward;
  }

  public void setMotorVoltage(double volts) {
    if (Constants.getRobot() == RobotType.SIMBOT) {
      m_elevatorSim.setInput(volts);
    } else {
      masterMotor.setVoltage(volts);
      leftSlaveFX.setVoltage(-volts);
    }
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

    Logger.recordOutput("Elevator Mech", m_Mechanism2d);
    Logger.recordOutput("Elevator Position", getElevatorHeight());
    Logger.recordOutput("Elevator Desired Position", elevatorPID.getSetpoint());

    if (Constants.getRobot() == RobotType.SIMBOT) {
      Logger.recordOutput("Elevator/SimPosition", m_elevatorSim.getPositionMeters());
    }
  }

  public void simulationPeriodicElevator() {
    if (Constants.getRobot() == RobotType.SIMBOT) {
      // Update the simulation with standard loop time
      m_elevatorSim.update(0.020);

      // Update encoder sim with new position from elevator sim
      m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());

      // Log the actual simulated position for debugging purposes
      Logger.recordOutput("Elevator/ActualPositionMeters", m_elevatorSim.getPositionMeters());
      Logger.recordOutput(
          "Elevator/ActualPositionInches", Units.metersToInches(m_elevatorSim.getPositionMeters()));

      // Update simulated motor position for real motor objects
      double simPositionRotations = Units.metersToInches(m_elevatorSim.getPositionMeters());
      masterMotor.setPosition(simPositionRotations);
      leftSlaveFX.setPosition(-simPositionRotations);

      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }
  }
}

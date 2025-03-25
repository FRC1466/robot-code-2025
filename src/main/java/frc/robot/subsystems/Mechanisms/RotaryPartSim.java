// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class RotaryPartSim extends SubsystemBase {
  // The P gain for the PID controller that drives this arm.
  private double m_armKp = 50;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getKrakenX60(1);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
  private final Encoder m_encoder = new Encoder(27, 28);
  private final PWMSparkMax m_motor = new PWMSparkMax(19);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          200,
          SingleJointedArmSim.estimateMOI(.25, 1.5),
          .25,
          Units.degreesToRadians(-360),
          Units.degreesToRadians(360),
          true,
          Constants.RotationConstants.restRadians,
          2.0 * Math.PI / 4096,
          0.0 // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 2);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 2, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              .25,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public RotaryPartSim() {
    m_encoder.setDistancePerPulse(2.0 * Math.PI / 4096);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  @Override
  public void periodic() {
    reachSetpoint();
  }

  @Override
  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint() {
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), -RobotContainer.rotaryPart.localSetpoint.getRadians());
    Logger.recordOutput("PidOutputSim", pidOutput);
    Logger.recordOutput("ArmSetpointSim", -RobotContainer.rotaryPart.localSetpoint.getDegrees());
    m_motor.setVoltage(pidOutput);
  }

  public double getAngleForVisualizationRads() {
    return m_armSim.getAngleRads();
  }

  public double getAngleForVisualizationDegs() {
    return 180 - Math.abs(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  public void stop() {
    m_motor.set(0.0);
  }
}

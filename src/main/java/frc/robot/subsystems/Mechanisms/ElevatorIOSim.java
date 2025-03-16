// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

/**
 * Implementation of ElevatorIO for simulation. Uses WPILib's ElevatorSim for physics simulation.
 */
public class ElevatorIOSim implements ElevatorIO {
  // Simulation constants
  private static final double DRUM_RADIUS_METERS = 0.02; // Radius of the elevator drum
  private static final double CARRIAGE_MASS_KG = 4.0; // Mass of the elevator carriage
  private static final double MIN_HEIGHT_METERS = 0.0; // Minimum elevator height
  private static final double MAX_HEIGHT_METERS = 1.5; // Maximum elevator height

  // Simulation objects
  private final ElevatorSim elevatorSim;
  private double appliedVolts = 0.0;
  private boolean isBrakeMode = true;

  public ElevatorIOSim() {
    // Create elevator simulation with 2 Kraken x60 motors
    // Adjust parameters to match your physical elevator's characteristics
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2), // Two Kraken x60 motors
            Constants.ElevatorConstants.elevatorPosition.gearRatio, // Gear reduction
            CARRIAGE_MASS_KG, // Carriage mass
            DRUM_RADIUS_METERS, // Drum radius
            MIN_HEIGHT_METERS, // Min height
            MAX_HEIGHT_METERS, // Max height
            true, // Simulate gravity
            0.0); // Initial position
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update the simulation with the last voltage command
    elevatorSim.setInputVoltage(appliedVolts);

    // Step the simulation by 20ms
    elevatorSim.update(0.02);

    // Update inputs with simulated values
    inputs.position = RobotContainer.elevator.elevatorPID.getSetpoint();
    inputs.velocity = elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmpsMaster = elevatorSim.getCurrentDrawAmps();
    inputs.currentAmpsSlave = elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void setNeutralMode(boolean isBrake) {
    this.isBrakeMode = isBrake;
    // In simulation, we don't need to do anything with this value,
    // but we store it in case we want to simulate coasting behavior in the future
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    // Set the simulated position (in meters)
    elevatorSim.setState(position, elevatorSim.getVelocityMetersPerSecond());
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms.elevator;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  public static final double carriageMassKg = Units.lbsToKilograms(6.0);
  public static final double stagesMassKg = Units.lbsToKilograms(12.0);
  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(2).withReduction(ElevatorIOTalonFX.reduction);

  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp
              / (gearbox.rOhms
                  * Math.pow(Elevator.drumRadius, 2)
                  * (carriageMassKg + stagesMassKg)
                  * gearbox.KvRadPerSecPerVolt));
  public static final Vector<N2> B =
      VecBuilder.fill(
          0.0, gearbox.KtNMPerAmp / (Elevator.drumRadius * (carriageMassKg + stagesMassKg)));

  // State given by elevator carriage position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double feedforward = 0.0;

  public ElevatorIOSim() {
    simState = VecBuilder.fill(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1khz
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(
            controller.calculate(simState.get(0) / Elevator.drumRadius) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.data =
        new ElevatorIOData(
            true,
            true,
            simState.get(0) / Elevator.drumRadius,
            simState.get(1) / Elevator.drumRadius,
            appliedVolts,
            Math.copySign(inputTorqueCurrent, appliedVolts),
            Math.copySign(inputTorqueCurrent, appliedVolts),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0);
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    setInputVoltage(volts);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(positionRad);
    this.feedforward = feedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    appliedVolts =
        gearbox.getVoltage(
            gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0) / Elevator.drumRadius);
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(1) / Elevator.drumRadius, voltage));
  }

  private void update(double dt) {
    inputTorqueCurrent =
        MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                A.times(x)
                    .plus(B.times(u))
                    .plus(
                        VecBuilder.fill(
                            0.0, -ElevatorConstants.G * ElevatorConstants.elevatorAngle.getSin())),
            simState,
            MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= 0.0) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, 0.0);
    }
    if (simState.get(0) >= ElevatorConstants.elevatorMaxTravel) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, ElevatorConstants.elevatorMaxTravel);
    }
  }

  @Override
  public void setEncoderPosition(double position) {
    simState.set(0, 0, position);
  }
}

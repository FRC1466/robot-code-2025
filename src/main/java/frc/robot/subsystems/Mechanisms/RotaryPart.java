// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems.Mechanisms;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.RotationConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import webblib.ArmPIDController;

public class RotaryPart extends SubsystemBase {
  public double armSetpoint = 0;

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteArmEncoder;
  private double peakOutput;
  private ArmPIDController armPID;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  @SuppressWarnings("unused")
  private double armPID_P;

  @SuppressWarnings("unused")
  private double armPID_output;

  @SuppressWarnings("unused")
  private double absoluteDistanceFromSpeaker;

  @AutoLogOutput public Rotation2d localSetpoint;
  private DoubleSupplier overrideFeedforward = () -> 0.0;
  private boolean disabled = false;
  private Rotation2d storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
  private boolean storedInPerimeter = false;
  private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
  private SysIdRoutine m_sysIdRoutine;

  public RotaryPart() {
    armMotor = new TalonFX(RotationConstants.armPort);

    peakOutput = RotationConstants.rotationPosition.peakOutput;
    absoluteArmEncoder = new DutyCycleEncoder(RotationConstants.dutyCyclePort);
    absoluteArmEncoder.setDutyCycleRange(0, 1);
    limitsConfigs.StatorCurrentLimit = 40;
    limitsConfigs.StatorCurrentLimitEnable = true;
    TalonFXConfigurator talonFXConfigurator = armMotor.getConfigurator();
    talonFXConfigurator.apply(limitsConfigs);
    // TODO: Remove this once it is completed
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

    // figure this out later
    // absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new ArmPIDController(
            RotationConstants.rotationPosition.P,
            RotationConstants.rotationPosition.I,
            RotationConstants.rotationPosition.D);
    armPID.setAvoidanceRange(
        Rotation2d.fromRadians(0), Rotation2d.fromRadians(RotationConstants.maxRadians));
    armPID.setTolerance(0.15);

    setGoal(Rotation2d.fromRadians(RotationConstants.restRadians));
    setDefaultCommand(hold());
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setTotalArmP(double p) {
    armPID_P = p;
    armPID.setP(p);
  }

  public Command setArmP(double p) {
    return runOnce(() -> setTotalArmP(p));
  }

  public Command setPeakOutput(double output) {
    return runOnce(() -> peakOutput = output);
  }

  public void setArmBrake() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command totalArmBrake() {
    return runOnce(() -> setArmBrake());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void simulationPeriodic() {}

  /** Configure arm motor. */

  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition = absoluteArmEncoder.get() / RotationConstants.dutyCycleResolution;
    return Rotation2d.fromRotations(initialPosition)
        .minus(Rotation2d.fromRotations(RotationConstants.absolutePositionOffset));
  }

  /**
   * Gets position of arm in radians. Assumes the arm being level within frame is the 0 point on the
   * x axis. Assumes CCW+.
   *
   * @return position in rad.
   */
  public Rotation2d getPosition() {
    return RotationConstants.encoderInverted
        ? getShiftedAbsoluteDistance().unaryMinus()
        : getShiftedAbsoluteDistance();
  }

  /**
   * Set arm with PID.
   *
   * @param setpoint setpoint in radians.
   */
  public void setGoal(Rotation2d setpoint) {
    Logger.recordOutput("Arm Setpoint", setpoint.getDegrees());
    localSetpoint = setpoint;
    armPID.setSetpoint(setpoint);
    armSetpoint = setpoint.getDegrees();
    // Logger.recordOutput("Arm PID Setpoint", setpoint.getRadians());
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(armPID.calculate(getPosition(), localSetpoint), -peakOutput, peakOutput);
    var feedforward = getPosition().getSin() * RotationConstants.gravityFF;
    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());

    // Logger.recordOutput("Arm PID Output", motorOutput);
    // Logger.recordOutput("Arm Feedforward", feedforward);
    // Logger.recordOutput("Arm Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setMotor(double percent) {
    if (percent > .35) {
      percent = .35;
    }
    armMotor.set(-percent);
  }

  public void setFeedforward(DoubleSupplier ff) {
    System.out.println("ff: " + ff.toString());
    overrideFeedforward = ff;
  }

  public Command store() {
    return runOnce(() -> setGoal(storedPosRad)).andThen(holdUntilSetpoint());
  }

  public Command coralScore() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(RotationConstants.coralPosRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command l4coralScore() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(RotationConstants.l4coralPosRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command algaeGrab() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(RotationConstants.algaePosition)))
        .andThen(holdUntilSetpoint());
  }

  public void setStoreSetpoint() {
    if (storedInPerimeter) {
      storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
    } else {
      storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
    }
    System.out.println("Override changed.");
    // Logger.recordOutput("In Frame Perimeter", storedInPerimeter);
    storedInPerimeter = !storedInPerimeter;
  }

  public Command hold() {
    return Commands.run(this::setArmHold, this);
  }

  public Command holdUntilSetpoint() {
    return hold()
        .raceWith(Commands.waitSeconds(0.3).andThen(Commands.waitUntil(this::isAtSetpoint)));
  }

  public Command toggleDisable() {
    return runOnce(
        () -> {
          disabled = !disabled;
        });
  }

  /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  @AutoLogOutput
  public boolean isAtSetpoint() {
    // Logger.recordOutput("Arm PID at setpoint", armPID.atSetpoint());
    return armPID.atSetpoint();
  }

  public void reset() {
    armPID.reset();
  }

  public boolean isAtStore() {
    return armPID.getSetpoint() == (Units.radiansToDegrees(RotationConstants.restRadians))
        && armPID.atSetpoint();
  }

  @Override
  public void periodic() {
    setArmHold();
    Logger.recordOutput("Arm Raw Absolute Encoder", absoluteArmEncoder.get());
    Logger.recordOutput("Arm Processed Absolute Encoder", getPosition().getRadians());
    Logger.recordOutput("Get Shifted Absolute Position", getShiftedAbsoluteDistance().getRadians());
    Logger.recordOutput("Get Arm P", armPID_P);
    Logger.recordOutput("Get Arm Output", peakOutput);
    Logger.recordOutput("Arm PID error", armPID.getPositionError());
    Logger.recordOutput("Arm Disabled", disabled);
  }
}

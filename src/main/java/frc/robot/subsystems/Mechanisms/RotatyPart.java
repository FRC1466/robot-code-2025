package frc.robot.subsystems.Mechanisms;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RotationConstants;
import webblib.ArmPIDController;

public class RotatyPart extends SubsystemBase{

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteArmEncoder;
  private double peakOutput;
  private ArmPIDController armPID;
  private double armPID_P;
  private double armPID_output;
  private double absoluteDistanceFromSpeaker;
  private double podiumRadians;
  private Rotation2d localSetpoint;
  private DoubleSupplier overrideFeedforward = () -> 0.0;
  private boolean disabled = false;
  private Rotation2d storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
  private boolean storedInPerimeter = false;


  public RotatyPart() {
    armMotor = new TalonFX(RotationConstants.armPort);

    
    peakOutput = RotationConstants.rotationPosition.peakOutput;
    absoluteArmEncoder = new DutyCycleEncoder(RotationConstants.dutyCyclePort);
    absoluteArmEncoder.setDutyCycleRange(0, 1);
    //figure this out later
    //absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new ArmPIDController(
            RotationConstants.rotationPosition.P,RotationConstants.rotationPosition.I, RotationConstants.rotationPosition.D);
    armPID.setAvoidanceRange(
        Rotation2d.fromRadians(RotationConstants.restRadians),
        Rotation2d.fromRadians(RotationConstants.maxRadians));
    armPID.setTolerance(0.15);

  /*   if (Robot.isSimulation()) {
      sim = new VirtualFourBarSimulation(absoluteArmEncoder);
      SmartDashboard.putData("Arm Sim", sim.getMech2d());
    }*/

  setGoal(Rotation2d.fromRadians(RotationConstants.restRadians));
  setDefaultCommand(hold());
  armMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void setTotalArmP(double p){
    armPID_P = p;
    armPID.setP(p);
  }
  public Command setArmP(double p){
    return runOnce(() -> setTotalArmP(p));
  }
    public Command setPeakOutput(double output){
    return runOnce(() -> peakOutput = output);
  }
  public void setArmBrake(){
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    
  }
  public Command totalArmBrake(){
    return runOnce(() -> setArmBrake());
  }
  @Override
  public void simulationPeriodic() {
    //sim.update(armMotor.get());
  }
  public void setPodiumRadians(double rad){
    podiumRadians = rad;
  }


  /** Configure arm motor. */

  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition =
        absoluteArmEncoder.get() / RotationConstants.dutyCycleResolution;
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
    localSetpoint = setpoint;
    armPID.setSetpoint(setpoint);
    SmartDashboard.putNumber("Arm PID Setpoint", setpoint.getRadians());
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            armPID.calculate(getPosition(), localSetpoint),
            -peakOutput,
            peakOutput);
    var feedforward = getPosition().getCos() * RotationConstants.gravityFF;
    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());

    SmartDashboard.putNumber("Arm PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setMotor(double percent) {
    armMotor.set(percent);

  }

  public void setFeedforward(DoubleSupplier ff) {
    System.out.println("ff: " + ff.toString());
    overrideFeedforward = ff;
  }

  public Command podium() {
    return run(() -> setGoal(Rotation2d.fromRadians(podiumRadians)))
       .andThen(holdUntilSetpoint());
  }


  


  public Command store() {
    return runOnce(() -> setGoal(storedPosRad))
       .andThen(holdUntilSetpoint());
  }

  public void setStoreSetpoint() {
    if (storedInPerimeter) {
      storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
    } else {
      storedPosRad = Rotation2d.fromRadians(RotationConstants.restRadians);
    }
    System.out.println("Override changed.");
    SmartDashboard.putBoolean("In Frame Perimeter", storedInPerimeter);
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
  public boolean isAtSetpoint() {
    SmartDashboard.putBoolean("Arm PID at setpoint", armPID.atSetpoint());
    return armPID.atSetpoint();
  }




  @Override
  public void periodic() {
   setArmHold();

    SmartDashboard.putData(absoluteArmEncoder);
    SmartDashboard.putNumber("PodiumRadians", podiumRadians);
    SmartDashboard.putNumber("Arm Raw Absolute Encoder", absoluteArmEncoder.get());
    SmartDashboard.putNumber("Arm Processed Absolute Encoder", getPosition().getRadians());
    SmartDashboard.putNumber("Get Shifted Absolute Position", getShiftedAbsoluteDistance().getRadians());
    SmartDashboard.putNumber("Get Arm P", armPID_P);
    SmartDashboard.putNumber("Get Arm Output", peakOutput);
    SmartDashboard.putNumber("Arm PID error", armPID.getPositionError());
    SmartDashboard.putBoolean("Arm Disabled", disabled);
}

}
// Copyright (c) 2025 FRC 1466
// https://github.com/FRC1466
package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swervedrive.Vision;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d RobotPose;

  private Command m_autonomousCommand;
  private Vision vision;
  private final RobotContainer m_robotContainer;

  private Timer timer = new Timer();
  private boolean limitSwitchCounter = false;

  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Test", Pose2d.struct).publish();

  public Robot() {

    m_robotContainer = new RobotContainer();
    AutoLogOutputManager.addObject(this); // Add this object for logging

    Logger.recordMetadata("ProjectName", "Robot-Code-2025"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
  }

  @Override
  public void robotInit() {
    m_robotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();
  }

  @Override
  public void robotPeriodic() {

    vision.logSeenAprilTags();
    // Color detectedColor = m_colorSensor.getColor();

    /*  if(m_robotContainer.limitSwitch.get() != limitSwitchCounter){
      if(limitSwitchCounter == false && m_robotContainer.elevator.getElevatorHeight() < 3){
        m_robotContainer.elevator.setSelectedSensorPosition(2.15);
      }
      limitSwitchCounter = m_robotContainer.limitSwitch.get();
    }*/

    var visionEst = RobotContainer.photonCamera.getEstimatedGlobalPose();
    //  SmartDashboard.putBoolean("booleanSwitch",m_robotContainer.limitSwitch.get());

    CommandScheduler.getInstance().run();
    RobotPose = RobotContainer.drivetrain.getState().Pose;
    visionEst.ifPresent(
        est -> {
          var estStdDevs = RobotContainer.photonCamera.getEstimationStdDevs();
          SmartDashboard.putNumber("Timestamp", est.timestampSeconds);
          SmartDashboard.putNumber(
              "Vision Pose X", visionEst.get().estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber(
              "Vision Pose Y", visionEst.get().estimatedPose.toPose2d().getY());
          // RobotContainer.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
          // est.timestampSeconds);    With Vision: With Vision:
          // addVisionMeasurement(est.estimatedPose.toPose2d(), Without Vision:
          // RobotPose.getRotation()), Without Vision: addVisionMeasurement(new
          // Pose2d(est.estimatedPose.toPose2d().getTranslation(), RobotPose.getRotation()),
          RobotContainer.drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });
    // SmartDashboard.pu Boolean("Camera?", photonCamera.getCamera().isConnected());
    SmartDashboard.putBoolean("Vision Est", visionEst.isPresent());

    SmartDashboard.putNumber("Robot Pose X", RobotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Robot Pose Y", RobotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber(
        "Robot Pose Theta", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    SwerveDriveState driveState = RobotContainer.drivetrain.getState();
    Logger.recordOutput("SwerveModuleStates", driveState.ModuleStates);
  }

  public void periodic() {}

  @Override
  public void disabledInit() {
    m_robotContainer.elevator.goToGoal(.5);
    // fix later
    //  m_robotContainer.rotatyPart.setGoal(Rotation2d.fromRadians(.05));
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.resetPID();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    timer.restart();
  }

  @Override
  public void teleopPeriodic() {
    /*SmartDashboard.putData
    ("Robot Pose", Telemetry.telemeterize.getPose());*/
    if (m_robotContainer.sliderEnabled) {
      m_robotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
    }
    Logger.recordOutput(
        "Elevator Slider Position", (((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 75));
    if (m_robotContainer.elevator.getElevatorHeight() < 7
        || m_robotContainer.elevator.getElevatorHeight() < 52) {
      m_robotContainer.elevator.setP(.05);
      m_robotContainer.elevator.setPeakOutput(.25);
    } else {
      m_robotContainer.elevator.setP(Constants.ElevatorConstants.elevatorPosition.P);
      m_robotContainer.elevator.setPeakOutput(
          Constants.ElevatorConstants.elevatorPosition.peakOutput);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

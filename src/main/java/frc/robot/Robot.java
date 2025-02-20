// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotType;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTester;
import frc.robot.subsystems.swervedrive.Vision;
import java.io.File;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;

  private double autoStart;
  private boolean autoMessagePrinted;

  private final Timer disabledTimer = new Timer();

  // AutoLog output for the estimated robot state pose.
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d RobotPose;

  private Command m_autonomousCommand;
  private Vision vision;
  private final RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private boolean limitSwitchCounter = false;

  @SuppressWarnings("unused")
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Test", Pose2d.struct).publish();

  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  private final Alert typeSwitchAlert =
      new Alert(
          "Cannot switch robot type while enabled. Disable the robot first.", AlertType.kWarning);

  public Robot() {
    m_robotContainer = new RobotContainer();
    AutoLogOutputManager.addObject(this); // Add this object for logging

    // Record metadata
    Logger.recordMetadata("ProjectName", "Robot-Code-2025");
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new RLOGServer());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new RLOGServer());
        break;

      case REPLAY:
        setUseTiming(false);
        File logsDir = new File("D:/logs");
        File latestLog =
            Stream.of(logsDir.listFiles())
                .filter(file -> file.getName().endsWith(".wpilog"))
                .max((f1, f2) -> Long.compare(f1.lastModified(), f2.lastModified()))
                .orElseThrow(() -> new RuntimeException("No .wpilog files found in D:/logs"));

        String logPath = latestLog.getAbsolutePath();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(logPath.replace(".wpilog", "_sim.wpilog")));
        break;
    }

    // Configure DriverStation for sim
    if (Constants.getRobot() == RobotType.SIMBOT) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Set up auto logging for RobotState
    AutoLogOutputManager.addObject(RobotState.class);

    // Start AdvantageKit Logger
    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(true);
    disabledTimer.restart();

    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 5);
  }

  @Override
  public void robotInit() {
    RobotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();
  }

  @Override
  public void robotPeriodic() {
    Constants.RobotType selectedType = m_robotContainer.getSelectedRobotType();
    if (selectedType != Constants.getRobot()) {
      if (DriverStation.isEnabled()) {
        // If robot is enabled, show warning and don't switch
        typeSwitchAlert.set(true);
        // Reset chooser to current type to prevent future attempts
        m_robotContainer.robotTypeChooser.addDefaultOption(
            Constants.getRobot().toString(), Constants.getRobot());
        // Reset chooser to current type to prevent future attempts
        m_robotContainer.robotTypeChooser.addDefaultOption(
            Constants.getRobot().toString(), Constants.getRobot());
      } else {
        // Only switch when disabled
        Constants.setRobot(selectedType);
        // Reinitialize drivetrain if robot type changes
        if (RobotContainer.drivetrain != null) {
          switch (selectedType) {
            case COMPBOT -> RobotContainer.drivetrain = TunerConstants.createDrivetrain();
            case DEVBOT -> RobotContainer.drivetrain = TunerConstantsTester.createDrivetrain();
            default -> throw new IllegalArgumentException("Unexpected value: " + selectedType);
          }
        }
      }
    }

    // Rest of robotPeriodic...
    vision.logSeenAprilTags();
    // Color detectedColor = m_colorSensor.getColor();

    /*
     if (m_robotContainer.limitSwitch.get() != limitSwitchCounter) {
       if (limitSwitchCounter == false && m_robotContainer.elevator.getElevatorHeight() < 3) {
         m_robotContainer.elevator.setSelectedSensorPosition(2.15);
       }
       limitSwitchCounter = m_robotContainer.limitSwitch.get();
     }
    */

    var visionEst = RobotContainer.photonCamera.getEstimatedGlobalPose();
    // SmartDashboard.putBoolean("booleanSwitch", m_robotContainer.limitSwitch.get());

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
          // With Vision: addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds,
          // stdDevs)
          // Without Vision: addVisionMeasurement(new
          // Pose2d(est.estimatedPose.toPose2d().getTranslation(), RobotPose.getRotation())
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

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
      // Leds.getInstance().lowBatteryAlert = true;
      // Useful when LEDs are implemented
    }

    // Print auto duration
    if (m_autonomousCommand != null) {
      if (!m_autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
    // Robot container periodic methods
    m_robotContainer.updateAlerts();
    m_robotContainer.updateDashboardOutputs();
  }

  public void periodic() {}

  @Override
  public void disabledInit() {
    RobotContainer.elevator.goToGoal(.5);
    // fix later
    // m_robotContainer.rotatyPart.setGoal(Rotation2d.fromRadians(.05));
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
    autoStart = Timer.getTimestamp();
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
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.sliderEnabled) {
      RobotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
    }
    Logger.recordOutput(
        "Elevator Slider Position", (((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 75));

    if (RobotContainer.elevator.getElevatorHeight() < 7
        || RobotContainer.elevator.getElevatorHeight() < 52) {
      RobotContainer.elevator.setP(.05);
      RobotContainer.elevator.setPeakOutput(.25);
    } else {
      RobotContainer.elevator.setP(Constants.ElevatorConstants.elevatorPosition.P);
      RobotContainer.elevator.setPeakOutput(
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

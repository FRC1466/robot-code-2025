// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotType;
import frc.robot.subsystems.Vision;
import frc.robot.util.Blinkin;
import frc.robot.util.LocalADStarAK;
import java.io.File;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final double loopOverrunWarningTimeout = 0.2;
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

  private Blinkin blinkin = new Blinkin();

  private Command m_autonomousCommand;
  private Vision vision;
  private final RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private Timer timer = new Timer();

  @SuppressWarnings("unused")
  private boolean checkState = false;

  @SuppressWarnings("unused")
  private boolean limitSwitchCounter = false;

  @SuppressWarnings("unused")
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Test", Pose2d.struct).publish();

  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  // Alert is no longer needed since we're commenting out the type switching functionality
  /*
  private final Alert typeSwitchAlert =
      new Alert(
          "Cannot switch robot type while enabled. Disable the robot first.", AlertType.kWarning);
  */

  @SuppressWarnings("resource")
  public Robot() {

    // Start logging! No more data receivers, replay sources, or metadata values may be added.

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
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

      case SIM:
        // Running a physics simulator, log to NT
        // Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter("C:/logs"));
        break;

      case REPLAY:
        setUseTiming(false);
        File logsDir = new File("D:/logs");
        File[] logFiles = logsDir.listFiles();
        if (logFiles == null) {
          throw new RuntimeException("Logs directory D:/logs does not exist or is not accessible");
        }
        File latestLog =
            Stream.of(logFiles)
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
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.notifyNewData();
    }

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Set up auto logging for RobotState
    AutoLogOutputManager.addObject(RobotState.class);

    DriverStation.silenceJoystickConnectionWarning(true);
    disabledTimer.restart();

    // Start AdvantageKit Logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void robotInit() {

    vision = new Vision();

    Pathfinding.setPathfinder(new LocalADStarAK());
    RobotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();
  }

  @Override
  public void robotPeriodic() {

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
    CommandScheduler.getInstance().run();
    RobotPose = RobotContainer.drivetrain.getState().Pose;
    visionEst.ifPresent(
        est -> {
          var estStdDevs = RobotContainer.photonCamera.getEstimationStdDevs();
          if (RobotContainer.visionEnabled) {
            // With vision - use full pose estimate
            RobotContainer.drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(est.timestampSeconds),
                estStdDevs);
          } else {
            // Without vision - maintain current rotation
            RobotContainer.drivetrain.addVisionMeasurement(
                new Pose2d(est.estimatedPose.toPose2d().getTranslation(), RobotPose.getRotation()),
                Utils.fpgaToCurrentTime(est.timestampSeconds),
                estStdDevs);
          }
        });

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);

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
    // Only run simulation-specific code for SIMBOT
    if (Constants.getRobot() == RobotType.SIMBOT) {
      System.out.println("Refreshing SIMBOT alliance configuration");
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.notifyNewData();
    }

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
  public void autonomousPeriodic() {
    blinkin.rainbowPartyLights();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.rotatyPart.coralScore();
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.sliderEnabled) {
      RobotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
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
  public void simulationPeriodic() {
    vision.simulationPeriodic(RobotContainer.drivetrain.getState().Pose);

    var debugField = vision.getSimDebugField();
    debugField
        .getObject("EstimatedRobot")
        .setPose(RobotPose = RobotContainer.drivetrain.getState().Pose);
  }
}

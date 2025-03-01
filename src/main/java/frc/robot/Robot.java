// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Vision;
import frc.robot.util.LocalADStarAK;
import java.io.File;
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

  // Map to track command counts for enhanced logging
  private final Map<String, Integer> commandCounts = new HashMap<>();

  @SuppressWarnings("resource")
  public Robot() {

    // Initialize vision here to avoid null reference in robotPeriodic
    vision = new Vision();

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
        File logsDir = new File("C:/logs");
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
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Set up auto logging for RobotState
    AutoLogOutputManager.addObject(RobotState.class);

    DriverStation.silenceJoystickConnectionWarning(true);
    disabledTimer.restart();

    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 5);

    // Start AdvantageKit Logger
    Logger.start();

    // Set up command logging to track what's happening with the command scheduler
    setupCommandLogging();
  }

  @Override
  public void robotInit() {

    // Remove redundant vision initialization since it's now in the constructor

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Add null check before accessing elevator
    if (RobotContainer.elevator != null) {
      RobotContainer.elevator.setSelectedSensorPosition(0);
    }

    // Check specifically for SIMBOT type, not just simulation
    if (Constants.getRobot() == RobotType.SIMBOT) {
      System.out.println("Detected SIMBOT - configuring Red1 alliance position");
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

    } else if (RobotBase.isSimulation()) {
      System.out.println("In simulation but not SIMBOT - skipping alliance configuration");
    }

    // Add diagnostic information
    System.out.println("Robot initialization complete");
    System.out.println("Robot type: " + Constants.getRobot());
    System.out.println("Joystick connected: " + DriverStation.isJoystickConnected(0));
  }

  // Create a timer for checking command status
  private final Timer commandMonitorTimer = new Timer();
  private boolean commandMonitorInitialized = false;

  @Override
  public void robotPeriodic() {

    // Initialize our monitoring timer if needed
    if (!commandMonitorInitialized) {
      commandMonitorTimer.reset();
      commandMonitorTimer.start();
      commandMonitorInitialized = true;
    }

    // Check command status every 500ms
    if (commandMonitorTimer.advanceIfElapsed(0.5)) {
      checkDriveCommand();
    }

    // Commented out robot type switching code
    /*
          Constants.RobotType selectedType = m_robotContainer.getSelectedRobotType();
          if (selectedType != Constants.getRobot()) {
            if (DriverStation.isEnabled()) {
              // If robot is enabled, show warning and don't switch
              typeSwitchAlert.set(true);
              // Reset chooser to current type to prevent future attempts
              m_robotContainer.robotTypeChooser.addDefaultOption(
                  Constants.getRobot().toString(), Constants.getRobot());
            } else {
              // Only switch when disabled
              Constants.setRobot(selectedType);
              // Reinitialize drivetrain if robot type changes
              if (RobotContainer.drivetrain != null) {
                switch (selectedType) {
                  case COMPBOT -> {
                    Constants.setMode(Mode.REPLAY);
                    RobotContainer.drivetrain = TunerConstants.createDrivetrain();
                  }
                  case DEVBOT -> {
                    Constants.setMode(Mode.REPLAY);
                    RobotContainer.drivetrain = TunerConstantsTester.createDrivetrain();
                  }
                  case SIMBOT -> {
                    Constants.setMode(Mode.SIM);
                    RobotContainer.drivetrain = TunerConstants.createDrivetrain();
                  }
                  default -> throw new IllegalArgumentException("Unexpected value: " + selectedType);
    =======
        Constants.RobotType selectedType = m_robotContainer.getSelectedRobotType();
        if (selectedType != Constants.getRobot()) {
          if (DriverStation.isEnabled()) {
            // If robot is enabled, show warning and don't switch
            typeSwitchAlert.set(true);
            // Reset chooser to current type to prevent future attempts
            m_robotContainer.robotTypeChooser.addDefaultOption(
                Constants.getRobot().toString(), Constants.getRobot());
          } else {
            // Only switch when disabled
            Constants.setRobot(selectedType);
            // Reinitialize drivetrain if robot type changes
            if (RobotContainer.drivetrain != null) {
              switch (selectedType) {
                case COMPBOT -> {
                  Constants.setMode(Mode.REPLAY);
                  RobotContainer.drivetrain = TunerConstants.createDrivetrain();

                }
              }
            }
        }
          */

    // Add null check before accessing vision
    if (vision != null) {
      vision.logSeenAprilTags();
    }

    // Add null check before accessing photonCamera
    if (RobotContainer.photonCamera != null) {
      var visionEst = RobotContainer.photonCamera.getEstimatedGlobalPose();

      // Only process vision estimate if drivetrain is initialized
      if (visionEst.isPresent() && RobotContainer.drivetrain != null) {
        var est = visionEst.get();
        var estStdDevs = RobotContainer.photonCamera.getEstimationStdDevs();
        SmartDashboard.putNumber("Timestamp", est.timestampSeconds);
        SmartDashboard.putNumber("Vision Pose X", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("Vision Pose Y", est.estimatedPose.toPose2d().getY());

        if (RobotContainer.visionEnabled) {
          // With vision - use full pose estimate
          RobotContainer.drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        } else if (RobotPose != null) {
          // Without vision - maintain current rotation
          RobotContainer.drivetrain.addVisionMeasurement(
              new Pose2d(est.estimatedPose.toPose2d().getTranslation(), RobotPose.getRotation()),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        }
      }
    }

    // Only update RobotPose if drivetrain is initialized
    if (RobotContainer.drivetrain != null) {
      RobotPose = RobotContainer.drivetrain.getState().Pose;
    }

    // Logger.recordOutput("apriltag seen", m_robotContainer.getClosestTag());

    // SmartDashboard.pu Boolean("Camera?", photonCamera.getCamera().isConnected());
    // SmartDashboard.putBoolean("Vision Est", visionEst.isPresent());

    // SmartDashboard.putNumber("Robot Pose X", RobotContainer.drivetrain.getState().Pose.getX());
    // SmartDashboard.putNumber("Robot Pose Y", RobotContainer.drivetrain.getState().Pose.getY());
    // SmartDashboard.putNumber(
    //     "Robot Pose Theta",
    // RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    // Add null check for SwerveDriveState
    @SuppressWarnings("unused")
    SwerveDriveState driveState =
        RobotContainer.drivetrain != null ? RobotContainer.drivetrain.getState() : null;

    // Logger.recordOutput("SwerveModuleStates", driveState.ModuleStates);

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
    // Ensure m_robotContainer is not null before calling methods on it
    if (m_robotContainer != null) {
      // Only call getClosestTag if drivetrain is not null to avoid cascading NullPointerExceptions
      if (RobotContainer.drivetrain != null) {
        // Logger.recordOutput("apriltag seen", m_robotContainer.getClosestTag());
      }

      // Robot container periodic methods - only call if not null
      m_robotContainer.updateAlerts();
      m_robotContainer.updateDashboardOutputs();
    }

    // Add diagnostic logging for drivetrain and joystick
    if (RobotContainer.drivetrain != null
        && m_robotContainer != null
        && m_robotContainer.joystick != null) {
      SmartDashboard.putNumber("Joystick Raw Y", m_robotContainer.joystick.getY());
      SmartDashboard.putNumber("Joystick Raw X", m_robotContainer.joystick.getX());
      SmartDashboard.putNumber("Joystick Raw Z", m_robotContainer.joystick.getZ());

      if (RobotBase.isReal()) {
        // Print diagnostic message every 100 cycles (approximately 2 seconds)
        if (lowBatteryCycleCount % 100 == 0) {
          System.out.println(
              "Drivetrain command: "
                  + (RobotContainer.drivetrain.getCurrentCommand() != null
                      ? RobotContainer.drivetrain.getCurrentCommand().getName()
                      : "None"));
          System.out.println(
              "Joystick values - Y: "
                  + m_robotContainer.joystick.getY()
                  + ", X: "
                  + m_robotContainer.joystick.getX()
                  + ", Z: "
                  + m_robotContainer.joystick.getZ());
        }
      }
    }

    // Run the CommandScheduler - make sure this happens in robotPeriodic
    CommandScheduler.getInstance().run();
  }

  // New method to check drive command status
  private void checkDriveCommand() {
    if (RobotContainer.drivetrain != null
        && DriverStation.isEnabled()
        && !DriverStation.isAutonomousEnabled()) {
      var defaultCommand = RobotContainer.drivetrain.getDefaultCommand();
      var currentCommand = RobotContainer.drivetrain.getCurrentCommand();

      SmartDashboard.putString(
          "Drivetrain Default Cmd", defaultCommand != null ? defaultCommand.getName() : "None");
      SmartDashboard.putString(
          "Drivetrain Current Cmd", currentCommand != null ? currentCommand.getName() : "None");

      // If default command exists but isn't running, force it to restart
      if (defaultCommand != null && currentCommand == null) {
        System.out.println("WARNING: Drive command not running - scheduling it now");
        if (m_robotContainer != null) {
          m_robotContainer.forceDriveCommand();
        } else if (defaultCommand != null) {
          defaultCommand.schedule();
        }
      }
    }

    // Additionally log active commands count to SmartDashboard
    if (RobotContainer.drivetrain != null) {
      int driveCommandCount = 0;
      for (Map.Entry<String, Integer> entry : commandCounts.entrySet()) {
        if (entry.getKey().contains("Drive") && entry.getValue() > 0) {
          driveCommandCount += entry.getValue();
        }
      }
      SmartDashboard.putNumber("Active Drive Commands", driveCommandCount);
    }
  }

  /** Setup enhanced command logging for diagnostic purposes */
  private void setupCommandLogging() {
    // Log active commands with unique identifiers
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);

          // Also log to console for important commands
          if (name.contains("Drive") || name.contains("Swerve")) {
            System.out.println((active ? "STARTED: " : "ENDED: ") + name + " - Count: " + count);
          }
        };

    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
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

    // Add null checks for elevator and rotatyPart
    if (RobotContainer.elevator != null) {
      RobotContainer.elevator.goToGoal(.5);
    }
    // fix later
    // m_robotContainer.rotatyPart.setGoal(Rotation2d.fromRadians(.05));
  }

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer != null) {
      m_robotContainer.resetPID();
    }
    // DO NOT cancel all commands here, as it could be causing our drive command issues
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Ensure m_robotContainer is not null before getting autonomous command
    if (m_robotContainer != null) {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      autoStart = Timer.getTimestamp();
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    } else {
      System.err.println("Warning: RobotContainer is null in autonomousInit");
    }
  }

  @Override
  public void autonomousPeriodic() {
    if (blinkin != null) {
      blinkin.rainbowPartyLights();
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Add null check before accessing rotatyPart
    if (RobotContainer.rotatyPart != null) {
      RobotContainer.rotatyPart.coralScore();
    }

    // Explicitly ensure drive command is enabled
    if (m_robotContainer != null) {
      System.out.println("Teleop init: Forcing drive command to schedule");
      m_robotContainer.forceDriveCommand();
    }

    // Also try scheduling directly
    if (RobotContainer.drivetrain != null) {
      var defaultCommand = RobotContainer.drivetrain.getDefaultCommand();
      if (defaultCommand != null) {
        System.out.println("Teleop init: Directly scheduling default command");
        defaultCommand.schedule();
      } else {
        System.out.println("WARNING: No default drive command to schedule in teleopInit");
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer
        .joystick
        .button(1)
        .whileTrue(
            m_robotContainer.m_pathfinder.getPathfindingCommand(
                0, m_robotContainer.getClosestTag()));
    // Skip auto-cancellation of commands in disabledPeriodic to prevent our drive command from
    // getting killed
    // by removing CommandScheduler.getInstance().cancelAll(); from disabledPeriodic

    // Add null check for blinkin and m_robotContainer
    if (blinkin != null && m_robotContainer != null) {
      if (m_robotContainer.getModeMethod()) {
        blinkin.coralLights();
      } else {
        blinkin.algaeLights();
      }
    }

    // Add null checks for elevator and joystick
    if (RobotContainer.sliderEnabled
        && RobotContainer.elevator != null
        && m_robotContainer != null
        && m_robotContainer.joystick != null) {
      RobotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
    }

    // Add null check for elevator
    if (RobotContainer.elevator != null) {
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
    // Add null checks before accessing vision and drivetrain
    if (vision != null && RobotContainer.drivetrain != null) {
      vision.simulationPeriodic(RobotContainer.drivetrain.getState().Pose);

      var debugField = vision.getSimDebugField();
      if (debugField != null) {
        debugField
            .getObject("EstimatedRobot")
            .setPose(RobotPose = RobotContainer.drivetrain.getState().Pose);
      }
    }
  }
}

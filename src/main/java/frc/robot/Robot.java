// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.util.MechanismVisualizer;
import frc.robot.util.rlog.RLOGServer;
import java.io.File;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static Robot instance;
  private static final double loopOverrunWarningTimeout = 0.2;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;

  private double autoStart;
  private boolean autoMessagePrinted;

  private final Timer disabledTimer = new Timer();

  private Blinkin blinkin = new Blinkin();

  private Command m_autonomousCommand;
  private Vision vision;
  private final RobotContainer m_robotContainer;

  // Combined mechanism visualizer for elevator and rotary part
  private MechanismVisualizer m_mechanismVisualizer;

  @SuppressWarnings("unused")
  private Timer algaeIntakeTimer = new Timer();

  @SuppressWarnings("unused")
  private boolean checkState = false;

  private DigitalInput beamBreak = new DigitalInput(9);

  private boolean lastBoolean = false;

  @SuppressWarnings("unused")
  private boolean limitSwitchCounter = false;

  private boolean ignoreJoystickInput = false;

  boolean allInputsNeutral = true;

  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  private final Alert flightstickNotCenteredAlert =
      new Alert(
          "Flightstick not centered! Center stick before using teleop controls.", AlertType.kError);

  // Flag to track if there are active warnings or errors
  private boolean activeWarnings = false;

  @SuppressWarnings("resource")
  public Robot() {
    instance = this;

    m_robotContainer = new RobotContainer();

    AutoLogOutputManager.addObject(this);

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
        // Running a physics simulator, log to NT4 and RLOG
        Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new NT4Publisher());
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

    Pathfinding.setPathfinder(new LocalADStarAK());
    RobotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();

    // Initialize the combined mechanism visualizer if in simulation mode

    m_mechanismVisualizer =
        new MechanismVisualizer(RobotContainer.elevator, RobotContainer.rotaryPartSim);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("AlgaeHeightReady?", RobotContainer.elevator.getElevatorHeight() > 20);
    RobotContainer.elevator.updateMechanism();
    Logger.recordOutput("Beam Break", beamBreak.get());
    /*if (!lastBoolean && beamBreak.get()) {
      RobotContainer.elevator.setSelectedSensorPosition(.5);
    }*/

    if (lastBoolean && !beamBreak.get()) {
      RobotContainer.elevator.setSelectedSensorPosition(.25);
    }
    lastBoolean = beamBreak.get();
    // Color detectedColor = m_colorSensor.getColor();

    /*
     if (m_robotContainer.limitSwitch.get() != limitSwitchCounter) {
       if (limitSwitchCounter == false && m_robotContainer.elevator.getElevatorHeight() < 3) {
         m_robotContainer.elevator.setSelectedSensorPosition(2.15);
       }
       limitSwitchCounter = m_robotContainer.limitSwitch.get();
     }
    */

    var visionEst = vision.getEstimatedGlobalPose();
    switch (Constants.getRobot()) {
      case DEVBOT, COMPBOT:
        visionEst.ifPresent(
            est -> {
              var estStdDevs = vision.getEstimationStdDevs();
              RobotContainer.drivetrain.addVisionMeasurement(
                  est.estimatedPose.toPose2d(),
                  Utils.fpgaToCurrentTime(est.timestampSeconds),
                  estStdDevs);
            });
        break;
      case SIMBOT:
        // Update the mechanism visualizer in simulation mode
        if (m_mechanismVisualizer != null) {
          m_mechanismVisualizer.periodic();
        }
        break;
      default:
        visionEst.ifPresent(
            est -> {
              var estStdDevs = vision.getEstimationStdDevs();
              RobotContainer.drivetrain.addVisionMeasurement(
                  est.estimatedPose.toPose2d(),
                  Utils.fpgaToCurrentTime(est.timestampSeconds),
                  estStdDevs);
            });
        break;
    }

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
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

    // Check for alerts on each robot periodic cycle
    checkAndHandleAlerts();
    CommandScheduler.getInstance().run();
  }

  public void periodic() {}

  @Override
  public void disabledInit() {
    RobotContainer.elevator.goToGoal(1);
    // fix later
    // m_robotContainer.rotaryPart.setGoal(Rotation2d.fromRadians(.05));
  }

  @Override
  public void disabledPeriodic() {
    /*  m_robotContainer.drivetrain.resetPose(
        new Pose2d(new Translation2d(7.277, 1.358), new Rotation2d(0)));

    if (m_robotContainer.getAutonomousCommand() instanceof PathPlannerAuto) {
      SmartDashboard.putNumber(
          "Starting Pose X",
          ((PathPlannerAuto) m_robotContainer.getAutonomousCommand()).getStartingPose().getX());
      SmartDashboard.putNumber(
          "Starting Pose Y",
          ((PathPlannerAuto) m_robotContainer.getAutonomousCommand()).getStartingPose().getY());
    }*/
    m_robotContainer.resetPID();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  public boolean shouldIgnoreJoystickInput() {
    return ignoreJoystickInput;
  }

  // get instance code
  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoStart = Timer.getTimestamp();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.rotaryPart.setGoal(
        Rotation2d.fromRadians(Constants.RotationConstants.coralPosRadians));
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

    // Reset joystick safety flag
    ignoreJoystickInput = false;

    allInputsNeutral = true;

    // Check if any joystick inputs are active
    boolean inputsActive = false;

    // Check main axes (with small deadband)
    if (Math.abs(m_robotContainer.joystick.getRawAxis(0)) > 0.1
        || // X axis
        Math.abs(m_robotContainer.joystick.getRawAxis(1)) > 0.1
        || // Y axis
        Math.abs(m_robotContainer.joystick.getRawAxis(2)) > 0.1) { // Twist axis
      inputsActive = true;
    }

    // Check all buttons (typically 1-12 for a standard flightstick)
    for (int i = 1; i <= 12; i++) {
      if (m_robotContainer.joystick.getHID().getRawButton(i)) {
        inputsActive = true;
        break;
      }
    }

    // Check all POVs (usually just one, but check all possible)
    for (int i = 0; i < m_robotContainer.joystick.getHID().getPOVCount(); i++) {
      if (m_robotContainer.joystick.getHID().getPOV(i) != -1) { // -1 means POV not pressed
        inputsActive = true;
        break;
      }
    }

    if (inputsActive) {
      // If any inputs are active, disable controls and set alert
      ignoreJoystickInput = true;
      flightstickNotCenteredAlert.set(true);
      DriverStation.reportError(
          "Flightstick not centered or buttons pressed! Controls disabled until all inputs are neutral.",
          false);
    } else {
      // All inputs are neutral
      flightstickNotCenteredAlert.set(false);
    }

    RobotContainer.rotaryPart.setGoal(
        Rotation2d.fromRadians(Constants.RotationConstants.coralPosRadians));
  }

  @Override
  public void teleopPeriodic() {
    // In teleopPeriodic, periodically check if all joystick inputs are neutral
    if (ignoreJoystickInput) {
      allInputsNeutral = true;

      // Check main axes
      if (Math.abs(m_robotContainer.joystick.getRawAxis(0)) > 0.1
          || Math.abs(m_robotContainer.joystick.getRawAxis(1)) > 0.1
          || Math.abs(m_robotContainer.joystick.getRawAxis(2)) > 0.1) {
        allInputsNeutral = false;
      }

      // Check all buttons
      for (int i = 1; i <= 12; i++) {
        if (m_robotContainer.joystick.getHID().getRawButton(i)) {
          allInputsNeutral = false;
          break;
        }
      }

      // Check all POVs (usually just one, but check all possible)
      for (int i = 0; i < m_robotContainer.joystick.getHID().getPOVCount(); i++) {
        if (m_robotContainer.joystick.getHID().getPOV(i) != -1) { // -1 means POV not pressed
          allInputsNeutral = false;
          break;
        }
      }

      if (allInputsNeutral) {
        // Re-enable joystick input
        ignoreJoystickInput = false;
        flightstickNotCenteredAlert.set(false);
        DriverStation.reportWarning("All controller inputs neutral. Controls re-enabled.", false);
      }
    }

    // Rest of your existing teleopPeriodic code
    if (RobotContainer.sliderEnabled) {
      RobotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
    }
  }

  @Override
  public void teleopExit() {
    ignoreJoystickInput = false;
    flightstickNotCenteredAlert.set(false);
    allInputsNeutral = true;
  }

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
    RobotContainer.elevator.simulationPeriodicElevator();
    vision.simulationPeriodic(RobotContainer.drivetrain.getState().Pose);

    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot");
  }

  // Check for active alerts of WARNING or ERROR type
  private void checkAndHandleAlerts() {
    // Check if there are any active warnings or errors
    boolean hasActiveAlerts = lowBatteryAlert.get() || flightstickNotCenteredAlert.get();

    // If warning state changed, update lights accordingly
    if (hasActiveAlerts != activeWarnings) {
      activeWarnings = hasActiveAlerts;

      if (activeWarnings) {
        // Activate warning lights when there are warnings/errors
        blinkin.warningLights();
      } else {
        // Return to normal light pattern when no warnings/errors
        if (RobotState.isAutonomous()) {
          blinkin.rainbowPartyLights();
        } else if (RobotState.isTeleop()) {
          blinkin.coralLights();
        }
      }
    }
  }
}

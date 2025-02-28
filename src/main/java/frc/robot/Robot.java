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
<<<<<<< Updated upstream
=======
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTester;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Vision;
import frc.robot.util.LocalADStarAK;
import java.io.File;
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
  private final Blinkin m_blinkin;

  @SuppressWarnings("unused")
  private Timer timer = new Timer();

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
    m_blinkin = new Blinkin();
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
  }

  @Override
  public void robotInit() {
    RobotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();

    Pathfinding.setPathfinder(new LocalADStarAK());
    RobotContainer.elevator.setSelectedSensorPosition(0);
    vision = new Vision();

    // Check specifically for SIMBOT type, not just simulation
    if (Constants.getRobot() == RobotType.SIMBOT) {
      System.out.println("Detected SIMBOT - configuring Red1 alliance position");
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

    } else if (RobotBase.isSimulation()) {
      System.out.println("In simulation but not SIMBOT - skipping alliance configuration");
    }
  }

  @Override
  public void robotPeriodic() {

<<<<<<< Updated upstream
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
>>>>>>> Stashed changes
            }
          }
        }
    }
      */

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
    Logger.recordOutput("apriltag seen", m_robotContainer.getClosestTag());

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
      m_blinkin.lightsWarning();
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
    blinkin.rainbowParty();
  }

  @Override
  public void autonomousPeriodic() {
    m_blinkin.lightsAuto();
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

    // SmartDashboard.putBoolean(
    //   "Drive Command Running", RobotContainer.drivetrain.getDefaultCommand().isScheduled());
    if (m_robotContainer.getModeMethod()) {
      blinkin.coralLights();
    } else {
      blinkin.alageLights();
    }
    if (RobotContainer.sliderEnabled) {
      RobotContainer.elevator.goToGoal(((m_robotContainer.joystick.getRawAxis(3) + 1) / 2) * 65);
      double radians = RobotContainer.rotatyPart.getPosition().getRadians();
      if (((radians > 0 && radians < .3) || (radians > -2 && radians < -1)) && checkState) {
        RobotContainer.rotatyPart.reset();
        RobotContainer.rotatyPart.setMotor(0);
        checkState = false;
      } else if ((radians > 0 && radians < .3) || (radians > -2 && radians < -1)) {

      } else {
        checkState = true;
      }
      Logger.recordOutput("Check State", checkState);
      Logger.recordOutput("Coral State", RobotContainer.boolCoralMode);
      /*SmartDashboard.putData
      ("Robot Pose", Telemetry.telemeterize.getPose());*/
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

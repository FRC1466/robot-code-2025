// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTester;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.RotatyPart;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.util.Pathfind;
import java.io.IOException;
import java.text.ParseException;
import java.util.Optional;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Warnings
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);

  // Maximum speed and angular rate
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Swerve drive platform control
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(.1)
          .withRotationalDeadband(.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  @SuppressWarnings("unused")
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Subsystems - declare as variables without initializing
  private Intake intake;
  public static RotatyPart rotatyPart;
  public static Elevator elevator;
  public static final Vision photonCamera = new Vision();

  // Always needed components
  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  final Pathfind m_pathfinder;

  @SuppressWarnings("unused")
  private Command m_pathfindCommandLeft;

  @SuppressWarnings("unused")
  private Command m_pathfindCommandRight;

  public static boolean visionEnabled = true;

  // Joystick and telemetry
  public final CommandJoystick joystick;
  final Telemetry logger;

  public static boolean coralMode = true;

  // Autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // Drivetrain
  public static CommandSwerveDrivetrain drivetrain;

  // State
  public static boolean sliderEnabled = false;

  // Store the drive command as instance variable so we can access it elsewhere
  private Command driveCommand;

  public RobotContainer() {
    // Initialize joystick first before any commands that might use it
    joystick = new CommandJoystick(0);
    logger = new Telemetry(MaxSpeed);

    // Initialize drivetrain and subsystems based on robot type
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        drivetrain = TunerConstants.createDrivetrain();
        Logger.recordOutput("Constant File", "Using TunerConstants for Compbot");

        // Initialize subsystems for COMPBOT
        intake = new Intake();
        elevator = new Elevator();
        rotatyPart = new RotatyPart();
      }
      case DEVBOT -> {
        drivetrain = TunerConstantsTester.createDrivetrain();
        Logger.recordOutput("Constant File", "Using TunerConstantsTester for Devbot");

        // Do NOT initialize subsystems for DEVBOT to avoid CAN errors
        // These remain null
      }
      case SIMBOT -> {
        drivetrain = TunerConstants.createDrivetrain();
        Logger.recordOutput("Constant File", "Using TunerConstants for simulation");

        // Initialize subsystems for SIMBOT
        intake = new Intake();
        elevator = new Elevator();
        rotatyPart = new RotatyPart();
      }
      default -> throw new IllegalArgumentException("Unexpected value: " + Constants.getRobot());
    }

    // Log controller connection status
    SmartDashboard.putBoolean("Controller Connected", DriverStation.isJoystickConnected(0));
    System.out.println("Controller connected: " + DriverStation.isJoystickConnected(0));

    // Only configure bindings if drivetrain is initialized
    if (drivetrain != null) {
      configureBindings();
    } else {
      System.err.println("WARNING: Drivetrain is null, skipping binding configuration");
    }

    try {
      m_pathfinder = new Pathfind(this);
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to initialize Pathfind", e);
    }

    // Force command registration to ensure drivetrain command gets set
    createDefaultDriveCommand();

    // Only initialize chooser if we're not in DEVBOT mode
    if (Constants.getRobot() != Constants.RobotType.DEVBOT) {
      initializeChooser();
    }
  }

  // New method to explicitly create the default drive command
  private void createDefaultDriveCommand() {
    if (drivetrain == null) {
      System.err.println("ERROR: Cannot create default drive command - drivetrain is null");
      return;
    }

    System.out.println("Creating drive command for " + Constants.getRobot());

    // Create a simple direct drive command for maximum reliability
    driveCommand =
        Commands.run(
            () -> {
              // Get joystick values with additional logging for debugging
              double yValue = joystick != null ? -joystick.getY() : 0;
              double xValue = joystick != null ? -joystick.getX() : 0;
              double zValue = joystick != null ? -joystick.getZ() : 0;

              // Apply deadband and calculate final drive values
              double finalY = Math.pow((MathUtil.applyDeadband(yValue, .05)), 3) * MaxSpeed;
              double finalX = Math.pow((MathUtil.applyDeadband(xValue, .05)), 3) * MaxSpeed;
              double finalZ = Math.pow((MathUtil.applyDeadband(zValue, .05)), 3) * MaxAngularRate;

              // Apply the drive values directly - bypass request pattern for simplicity
              SwerveRequest.FieldCentric request =
                  new SwerveRequest.FieldCentric()
                      .withVelocityX(finalY) // Forward/back
                      .withVelocityY(finalX) // Left/right
                      .withRotationalRate(finalZ) // Rotation
                      .withDeadband(0.05)
                      .withRotationalDeadband(0.05)
                      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

              // Set control directly
              drivetrain.setControl(request);

              // Log values to dashboard
              SmartDashboard.putNumber("Drive Y (Fwd/Rev)", finalY);
              SmartDashboard.putNumber("Drive X (Left/Right)", finalX);
              SmartDashboard.putNumber("Drive Z (Rotation)", finalZ);
            },
            drivetrain);

    // Name the command for easier debugging
    driveCommand.setName("DirectDriveCommand");

    // Add command name and requirements logging
    System.out.println("Creating drive command named: " + driveCommand.getName());
    System.out.println(
        "Command requirements: "
            + driveCommand.getRequirements().stream()
                .map(subsystem -> subsystem.getClass().getSimpleName())
                .collect(Collectors.joining(", ")));

    // Set as default command for the drivetrain
    drivetrain.setDefaultCommand(driveCommand);

    System.out.println("Drive command created: " + driveCommand.getName());
    Logger.recordOutput("DriveCommand/Name", driveCommand.getName());
    Logger.recordOutput("DriveCommand/HashCode", Integer.toHexString(driveCommand.hashCode()));

    // Force schedule the command to ensure it starts immediately
    if (driveCommand != null && !driveCommand.isScheduled()) {
      driveCommand.schedule();
      System.out.println("Drive command scheduled: " + driveCommand.getName());
      Logger.recordOutput("DriveCommand/ScheduledManually", true);
    }

    // Log the successful setup
    SmartDashboard.putString("Drive Command Status", "Created: " + driveCommand.getName());
  }

  // Initialize autonomous chooser with options
  public void initializeChooser() {
    // Check if required subsystems are initialized before registering commands
    if (elevator == null || intake == null || rotatyPart == null) {
      System.out.println("Warning: Some subsystems are null, skipping named command registration");
      return;
    }

    // Only register commands if we have the necessary subsystems
    NamedCommands.registerCommand("Raise arm to Station", elevator.toStation());
    NamedCommands.registerCommand("raise arm to 1", elevator.toL1());
    NamedCommands.registerCommand("raise arm to 2", elevator.toL2());
    NamedCommands.registerCommand("raise arm to 3", elevator.toL3());
    NamedCommands.registerCommand("raise arm to 4", elevator.toL4());
    NamedCommands.registerCommand("intake coral", intake.intake());
    NamedCommands.registerCommand("reverse intake coral", intake.reverseIntake());
    NamedCommands.registerCommand("rotary part", rotatyPart.coralScore());

    autoChooser.addOption("2 Piece", new PathPlannerAuto("2 Piece Auto"));
    autoChooser.addOption("2 Piece Inverted", new PathPlannerAuto("Invert 2 Piece Auto"));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Piece Auto Better"));
    autoChooser.addOption("Recenter bot", new PathPlannerAuto("Recenter"));
    autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));

    // Publish the chooser to the dashboard
    SmartDashboard.putData("Auto Selector", autoChooser.getSendableChooser());
  }

  // Configure joystick bindings
  @SuppressWarnings("unused")
  private void configureBindings() {
    switch (Constants.getRobot()) {
      case COMPBOT -> configureCompbotBindings();
      case DEVBOT -> configureDevbotBindings();
      case SIMBOT -> configureCompbotBindings(); // SIMBOT uses same bindings as COMPBOT
    }
  }

  // Configure bindings for COMPBOT and SIMBOT with all subsystems
  private void configureCompbotBindings() {
    // Check if drivetrain is null before setting command
    if (drivetrain == null) {
      System.err.println("ERROR: Cannot configure COMPBOT bindings - drivetrain is null");
      return;
    }

    // Check if joystick is connected
    if (!DriverStation.isJoystickConnected(0)) {
      System.err.println("WARNING: Joystick on port 0 is not connected!");
      SmartDashboard.putString("Drive Status", "NO CONTROLLER");
    }

    System.out.println("Setting up COMPBOT bindings");

    // IMPORTANT: DO NOT set the default command here
    // We handle this in createDefaultDriveCommand()

    // Reset the field-centric heading with vision enabled
    joystick
        .povDown()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = true;
                  drivetrain.seedFieldCentric();
                }));

    // Reset the field-centric heading with vision disabled
    joystick
        .povUp()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = false;
                  drivetrain.seedFieldCentric();
                }));

    // Mode Switch
    joystick.button(2).onTrue(changeMode());

    // ----------------------------------------------
    // SUBSYSTEM COMMANDS (ELEVATOR, INTAKE, ETC)
    // ----------------------------------------------
    // Initialize triggers for subsystem commands
    Trigger intakeProximityTrigger = new Trigger(() -> intake.getIntakeDistanceBool());
    Trigger algaeHeightReady = new Trigger(() -> elevator.getElevatorHeight() > 20);
    Trigger currentIntakeSwitch = new Trigger(() -> intake.getHighCurrent());
    Trigger algaeMode = new Trigger(() -> getModeMethod());
    Trigger coralMode = new Trigger(() -> !getModeMethod());
    @SuppressWarnings("unused")
    Trigger falseIntakeProximityTrigger = new Trigger(() -> !intake.getIntakeDistanceBool());

    // Elevator and rotary part controls
    joystick.button(11).onTrue(elevator.toL2()).onFalse(elevator.toBottom());
    joystick.button(12).onTrue(rotatyPart.coralScore());

    // Intake Coral
    joystick
        .button(3)
        .and(intakeProximityTrigger)
        .whileTrue(intake.intake().alongWith(rotatyPart.store()).alongWith(elevator.toBottom()))
        .onFalse(
            Commands.waitSeconds(.07).andThen(intake.stop()).alongWith(rotatyPart.coralScore()));

    // L2
    joystick
        .button(5)
        .and(coralMode)
        .onTrue(elevator.toL2().alongWith(rotatyPart.coralScore()))
        .onFalse(intake.outTake());

    (intakeProximityTrigger).onTrue(elevator.toBottom().andThen(intake.stop()));

    // L3
    joystick
        .button(6)
        .and(coralMode)
        .onTrue(elevator.toL3().alongWith(rotatyPart.coralScore()))
        .onFalse(intake.intake());

    // L4 and Algae mode controls
    joystick
        .button(1)
        .and(algaeMode)
        .onTrue(intake.outTake())
        .onFalse(rotatyPart.coralScore().alongWith(elevator.toBottom()));

    joystick
        .button(6)
        .and(algaeMode)
        .onTrue(rotatyPart.coralScore().alongWith(elevator.toL2Algae()));

    joystick
        .button(6)
        .and(algaeHeightReady)
        .onTrue(rotatyPart.algaeGrab().alongWith(intake.reverseIntake()));

    joystick
        .button(6)
        .and(algaeMode)
        .and(currentIntakeSwitch)
        .onFalse(elevator.toL2().alongWith(intake.algaeHold()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // Configure bindings for DEVBOT with minimal functionality
  private void configureDevbotBindings() {
    // Check if drivetrain is null before configuring
    if (drivetrain == null) {
      System.err.println("ERROR: Cannot configure DEVBOT bindings - drivetrain is null");
      return;
    }

    // Check if joystick is connected
    if (!DriverStation.isJoystickConnected(0)) {
      System.err.println("WARNING: Joystick on port 0 is not connected!");
      SmartDashboard.putString("Drive Status", "NO CONTROLLER");
    }

    System.out.println("Setting up DEVBOT bindings");

    // IMPORTANT: DO NOT set the default command here
    // We handle this in createDefaultDriveCommand()

    // Add a button that can manually activate the drive command for testing
    joystick
        .button(1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("Button 1 pressed - ensuring drive command is scheduled");
                  if (driveCommand != null && !driveCommand.isScheduled()) {
                    driveCommand.schedule();
                  }
                }));

    // Reset the field-centric heading with vision enabled
    joystick
        .povDown()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = true;
                  drivetrain.seedFieldCentric();
                }));

    // Reset the field-centric heading with vision disabled
    joystick
        .povUp()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = false;
                  drivetrain.seedFieldCentric();
                }));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // Switch state command
  public Command switchState(boolean bool) {
    return run(() -> changeState(bool));
  }

  // Change state method
  public void changeState(boolean bool) {
    sliderEnabled = bool;
  }

  // Get autonomous command
  public Command getAutonomousCommand() {
    if (autoChooser == null) {
      System.out.println("Warning: autoChooser is null, returning no-op command");
      return Commands.none(); // Return an empty command instead of null
    }
    return autoChooser.get();
  }

  public Command changeMode() {
    return Commands.runOnce(() -> changeModeMethod());
  }

  public void changeModeMethod() {
    coralMode = !coralMode;
  }

  public boolean getModeMethod() {
    return coralMode;
  }

  // Reset PID controllers - only if they exist
  public void resetPID() {
    if (elevator != null) {
      elevator.reset();
    }
    if (rotatyPart != null) {
      rotatyPart.reset();
    }
  }

  // Update alerts
  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(joystick.getHID().getPort()));
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Add optional status indicators that are useful for debugging
    SmartDashboard.putBoolean("Drivetrain Available", drivetrain != null);
    SmartDashboard.putBoolean("Elevator Available", elevator != null);
    SmartDashboard.putBoolean("Vision Available", photonCamera != null);

    // Add controller diagnostic information
    SmartDashboard.putBoolean("Controller Connected", DriverStation.isJoystickConnected(0));

    // Add drive command status
    if (drivetrain != null) {
      var defaultCommand = drivetrain.getDefaultCommand();
      var currentCommand = drivetrain.getCurrentCommand();

      SmartDashboard.putBoolean("Drive Command Available", driveCommand != null);
      SmartDashboard.putBoolean(
          "Drive Command Scheduled", driveCommand != null && driveCommand.isScheduled());
      SmartDashboard.putString(
          "Default Command", defaultCommand != null ? defaultCommand.getName() : "None");
      SmartDashboard.putString(
          "Current Command", currentCommand != null ? currentCommand.getName() : "None");

      // Log robot movement values for debugging
      if (currentCommand != null && currentCommand.isScheduled()) {
        SmartDashboard.putString("Drive Status", "ACTIVE");
      } else {
        SmartDashboard.putString("Drive Status", "INACTIVE");
      }
    } else {
      SmartDashboard.putString("Drivetrain Status", "NULL");
    }
  }

  // Add a method to force schedule the drive command
  public void forceDriveCommand() {
    if (driveCommand != null && !driveCommand.isScheduled()) {
      System.out.println("Forcing drive command to schedule: " + driveCommand.getName());
      driveCommand.schedule();
      Logger.recordOutput("DriveCommand/ForcedSchedule", true);
    } else if (driveCommand == null) {
      System.out.println("WARNING: Cannot force null drive command");
      Logger.recordOutput("DriveCommand/NullCommand", true);
    } else {
      System.out.println("Drive command already scheduled: " + driveCommand.getName());
      Logger.recordOutput("DriveCommand/AlreadyScheduled", true);
    }
  }

  // Should probably get an error handler and some data printing
  /**
   * @return int from 0-5 , if blue 0 = 17 and 5 = 22 , if red 0 = 6 and 5 = 11
   */
  public int getClosestTag() {
    int closestTag = -1;
    double prevDistance = Double.MAX_VALUE;
    double holderDistance = 0;
    int offset = 0;

    // Check for null drivetrain before attempting to access it
    if (drivetrain == null) {
      return -1; // Return -1 if drivetrain is null
    }

    // Get alliance safely with null check
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        offset = 6;
      } else if (alliance.get() == Alliance.Blue) {
        offset = 17;
      }
    }

    try {
      for (int i = offset; i < (offset + 6); i++) {
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(i);
        if (tagPoseOptional.isEmpty()) {
          // Skip this index if no tag pose is available
          continue;
        }
        var tagPose = tagPoseOptional.get();
        holderDistance =
            (Math.pow(drivetrain.getState().Pose.getX() - tagPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - tagPose.getY(), 2)
                + Math.pow(
                    drivetrain.getState().Pose.getRotation().getRadians()
                        - tagPose.getRotation().getAngle() * 0.1,
                    2));
        if (holderDistance < prevDistance) {
          closestTag = i;
          prevDistance = holderDistance;
        }
      }
    } catch (Exception e) {
      // Retrieve the actual exception thrown by the invoked method
      Throwable cause = e.getCause();
      System.err.println("The underlying exception was: " + cause);
      e.printStackTrace();
    }
    return closestTag - offset;
  }
}

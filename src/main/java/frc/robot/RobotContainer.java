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
import org.littletonrobotics.junction.AutoLogOutput;
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

  // Autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // Add new field
  public final LoggedDashboardChooser<Constants.RobotType> robotTypeChooser =
      new LoggedDashboardChooser<>("Robot Type");

  // Subsystems
  private final Intake intake = new Intake();
  public static final RotatyPart rotatyPart = new RotatyPart();
  public static final Vision photonCamera = new Vision();
  public static final Elevator elevator = new Elevator(rotatyPart);

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  final Pathfind m_pathfinder;

  @SuppressWarnings("unused")
  private Command m_pathfindCommandLeft;

  @SuppressWarnings("unused")
  private Command m_pathfindCommandRight;

  @AutoLogOutput public static boolean visionEnabled = true;

  // Joystick and telemetry
  public final CommandJoystick joystick = new CommandJoystick(0);
  final Telemetry logger = new Telemetry(MaxSpeed);

  @AutoLogOutput public static boolean coralMode = true;

  // Drivetrain
  public static CommandSwerveDrivetrain drivetrain;
  // State
  public static boolean sliderEnabled = false;

  public RobotContainer() {
    configureRobotTypeChooser();
    // Initialize drivetrain based on robot type
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        drivetrain = TunerConstants.createDrivetrain();
        Logger.recordOutput("Constant File", "Using TunerConstants for Compbot");
      }
      case DEVBOT -> {
        drivetrain = TunerConstantsTester.createDrivetrain();
        Logger.recordOutput("Constant File", "Using TunerConstantsTester for Devbot");
      }
      case SIMBOT -> {
        drivetrain = TunerConstants.createDrivetrain(); // Use TunerConstants for simulation
        Logger.recordOutput("Constant File", "Using TunerConstants for simulation");
      }
      default -> throw new IllegalArgumentException("Unexpected value: " + Constants.getRobot());
    }
    try {
      m_pathfinder = new Pathfind(this);
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to initialize Pathfind", e);
    }

    configureBindings();
    initializeChooser();
  }

  private void configureRobotTypeChooser() {
    robotTypeChooser.addOption("COMPBOT", Constants.RobotType.COMPBOT);
    robotTypeChooser.addOption("DEVBOT", Constants.RobotType.DEVBOT);
    robotTypeChooser.addOption("SIMBOT", Constants.RobotType.SIMBOT);

    // Set default to current robot type
    robotTypeChooser.addDefaultOption(Constants.getRobot().toString(), Constants.getRobot());
  }

  // Add getter method
  public Constants.RobotType getSelectedRobotType() {
    return robotTypeChooser.get();
  }

  // Initialize autonomous chooser with options
  public void initializeChooser() {
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
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        Math.pow((MathUtil.applyDeadband(-joystick.getY(), .05)), 3)
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        Math.pow((MathUtil.applyDeadband(-joystick.getX(), .05)), 3)
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        Math.pow((MathUtil.applyDeadband(-joystick.getZ(), .05)), 3)
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    Trigger intakeProximityTrigger = new Trigger(() -> intake.getIntakeDistanceBool());
    Trigger algaeHeightReady = new Trigger(() -> elevator.getElevatorHeight() > 20);
    Trigger currentIntakeSwitch = new Trigger(() -> intake.getHighCurrent());
    Trigger algaeMode = new Trigger(() -> getModeMethod());
    Trigger coralMode = new Trigger(() -> !getModeMethod());
    Trigger falseIntakeProximityTrigger = new Trigger(() -> !intake.getIntakeDistanceBool());

    // reset the field-centric heading on left bumper press
    joystick
        .povDown()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = true;
                  drivetrain.seedFieldCentric();
                  // Add debug output
                  SmartDashboard.putBoolean("Reset Complete", true);
                }));

    joystick
        .povUp()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  visionEnabled = false;
                  drivetrain.seedFieldCentric();
                  // Add debug output
                  SmartDashboard.putBoolean("Reset Complete", true);
                }));

    // Intake Coral
    // joystick.button(1).and(intakeProximityTrigger).whileTrue(elevator.toBottom().alongWith(rotatyPart.store()).alongWith(intake.intake())).onFalse(intake.stop().alongWith(rotatyPart.coralScore()));
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // Reset the field-centric heading on left bumper press
    // Mode Switch
    joystick.button(2).onTrue(changeMode());
    // Intake Coral
    joystick
        .button(3)
        .and(coralMode)
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
    // L4

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

  // Reset PID controllers
  public void resetPID() {
    elevator.reset();
    rotatyPart.reset();
  }

  // Update alerts
  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(joystick.getHID().getPort()));
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  // Should probably get an error handler and some data printing
  /**
   * @return int from 0-5 , if blue 0 = 17 and 5 = 22 , if red 0 = 6 and 5 = 11
   */
  @AutoLogOutput(key = "Logger/closestTag")
  public int getClosestTag() {
    int closestTag = -1;
    double prevDistance = Double.MAX_VALUE;
    double holderDistance = 0;
    int offset = 0;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      offset = 6;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      offset = 17;
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
    Logger.recordOutput("Logger/closestTagID", closestTag);
    Logger.recordOutput("current offset is", offset);
    Logger.recordOutput("Alliance is Red", DriverStation.getAlliance().get() == Alliance.Red);
    Logger.recordOutput("Alliance is Blue", DriverStation.getAlliance().get() == Alliance.Blue);
    // Adjust the tag based on alliance.
    return closestTag - offset;
  }
}

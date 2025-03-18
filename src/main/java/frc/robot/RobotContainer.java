// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.PathfindConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTester;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.RotaryPart;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.util.FlipField;
import frc.robot.util.Pathfind;
import java.io.IOException;
import java.text.ParseException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Command reefCommand = null;
  private Command algaeCommand = null;
  private Command stationCommand = null;

  // Warnings
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert TeleopPaused =
      new Alert("All Teleop commands cancelled.", AlertType.kWarning);

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

  // Autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // Subsystems
  private final Intake intake = new Intake();
  public static final RotaryPart rotaryPart = new RotaryPart();
  public static final Vision photonCamera = new Vision();
  public static final Elevator elevator = new Elevator();

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  final Pathfind m_pathfinder;

  public static boolean visionEnabled = true;

  public static int leftCoral = 1;

  public static double gyroMultiplier = 0;

  public static boolean pathfindingOverride = false;

  // Joystick and telemetry
  public final CommandJoystick joystick = new CommandJoystick(0);

  final Telemetry logger = new Telemetry(MaxSpeed);
  @AutoLogOutput public static boolean algaeMode = false;

  public static CommandSwerveDrivetrain drivetrain;
  public static boolean sliderEnabled = false;

  // Kill switch for autonomous pathing
  @AutoLogOutput public static boolean autoPathingEnabled = true;
  private final edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Boolean>
      pathingEnabledChooser = new edu.wpi.first.wpilibj.smartdashboard.SendableChooser<>();
  private final edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Boolean> testingChooser =
      new edu.wpi.first.wpilibj.smartdashboard.SendableChooser<>();
  private boolean testingBoolean = false;

  public RobotContainer() {
    // Initialize drivetrain once based on robot type
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
        drivetrain = TunerConstants.createDrivetrain();
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
    configureTestingBindings();
    initializeChooser();
    // Initialize the pathing kill switch chooser
    setupTestingChooser();
    setupPathingKillSwitch();
  }

  // Initialize autonomous chooser with options
  public void initializeChooser() {
    BooleanSupplier armRaiseReefPositionCheck =
        () -> !autoPathingEnabled || armFieldReady(leftCoral, 1);
    Trigger conditionalArmRaiseReefReady = new Trigger(armRaiseReefPositionCheck);
    // Create the Intake command that runs for exactly 2 seconds
    Command intakeHeightCommand = elevator.toBottom().alongWith(rotaryPart.store());
    Command intakeCommand =
        intake.intake().withTimeout(3).andThen(rotaryPart.coralScore().alongWith(intake.stop()));

    // L2 command - go to position, outtake, hold for 2 seconds, then return to bottom
    Command CoralScoreCommand = rotaryPart.coralScore();
    Command l2HeightCommand = elevator.toL2();

    Command l2ScoreCommand =

        elevator
            .toL2()
            .andThen(intake.outTake())
            .alongWith(Commands.waitSeconds(2))
            .andThen(elevator.toBottom().alongWith(rotaryPart.coralScore()));

    // L3 command - go to position, outtake, hold for 2 seconds, then return to bottom
    Command l3Command =
        elevator
            .toL3()
            .alongWith(rotaryPart.coralScore())
            .alongWith(Commands.waitSeconds(1))
            .andThen(intake.outTake().withTimeout(1))
            .andThen(elevator.toBottom().alongWith(rotaryPart.coralScore()));

    // Create the L4 command with conditional follow-up action when elevator reaches position

    Command l4ScoreCommand =
        Commands.waitUntil(() -> elevator.getElevatorHeight() > 61)
            .andThen(intake.outTake())
            .alongWith(Commands.waitSeconds(1))
            .andThen(intake.stop())
            .alongWith(rotaryPart.coralScore())
            .alongWith(elevator.toBottom());
    Command l4RaiseCommand = elevator.toL4();

    /* .until(() -> elevator.getElevatorHeight() > 58)
    .andThen(rotaryPart.l4coralScore())
    .alongWith(intake.coralHold());*/
    /*    .andThen(intake.outTake().withTimeout(.5))
    .andThen(elevator.toBottom().alongWith(rotaryPart.coralScore()))*/
    // Register the named commands
    NamedCommands.registerCommand("CoralScore", rotaryPart.coralScore());
    NamedCommands.registerCommand("IntakeElevator", intakeHeightCommand);
    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand(
        "l2Elevator", Commands.waitUntil(armRaiseReefPositionCheck).andThen(elevator.toL2()));
    NamedCommands.registerCommand("l2Score", l2ScoreCommand);
    NamedCommands.registerCommand("l3", l3Command);
    NamedCommands.registerCommand("l4Raise", l4RaiseCommand);
    NamedCommands.registerCommand("l4Score", l4ScoreCommand);
    // autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi Auto"));
    autoChooser.addOption("Right Taxi Auto", new PathPlannerAuto("Right Taxi Auto"));
    autoChooser.addOption("Left Taxi Auto", new PathPlannerAuto("Left Taxi Auto"));
    autoChooser.addOption("Right 1 Piece", new PathPlannerAuto("Right One Piece"));
    autoChooser.addOption("Left 1 Piece", new PathPlannerAuto("Left One Piece"));
    /*  autoChooser.addOption("2 Piece", new PathPlannerAuto("2 Piece Auto"));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Piece Auto"));
    autoChooser.addOption("4 Piece", new PathPlannerAuto("4 Piece Auto Faster"));
    autoChooser.addOption("5 Piece", new PathPlannerAuto("5 Piece Auto Faster"));*/
    /*autoChooser.addOption(
        "L4 Testing",
        CoralScoreCommand.andThen(pathfindingAutoFactory(0, 3, true))
            .alongWith(l4RaiseCommandFactory(0, 3))); /*
    /  .alongWith(l4RaiseCommandFactory(0, 3))
    .alongWith(l4ScoreCommand));*/

    // Publish the chooser to the dashboard
    SmartDashboard.putData("Auto Selector", autoChooser.getSendableChooser());
  }

  public Command pathfindingAutoFactory(int leftRight, int tagTo, boolean L4) {
    return Commands.runOnce(
        () -> {
          if (autoPathingEnabled) {
            if (L4) {
              reefCommand = m_pathfinder.getPathfindingCommandReefL4(leftRight, tagTo);
            } else {
              reefCommand = m_pathfinder.getPathfindingCommandReef(leftRight, tagTo);
            }
            reefCommand.schedule();
          }
        });
  }

  public Command l4RaiseCommandFactory(int leftRight, int tagTo) {
    BooleanSupplier armRaisePositionCheck = () -> armFieldAutoReady(leftRight, 1, tagTo);
    Trigger conditionalRaiseReefReady = new Trigger(armRaisePositionCheck);
    BooleanSupplier armReefPositionCheck = () -> armFieldAutoReady(leftRight, 1, tagTo);
    Trigger conditionalArmReefReady = new Trigger(armReefPositionCheck);
    return Commands.waitUntil(conditionalArmReefReady)
        .andThen(
            rotaryPart
                .coralScore()
                .alongWith(elevator.toL4())
                .alongWith(Commands.waitUntil(() -> elevator.getElevatorHeight() > 58))
                .andThen(Commands.waitUntil(conditionalRaiseReefReady))
                .andThen(rotaryPart.l4coralScore())
                .alongWith(intake.coralHold())
                .alongWith(Commands.waitUntil(conditionalArmReefReady))
                .andThen(intake.outTake())
                .alongWith(Commands.waitSeconds(1))
                .andThen(intake.stop())
                .alongWith(rotaryPart.coralScore())
                .alongWith(elevator.toBottom()));
  }

  /*   safeButton5
      .and(coralMode)
      .and(conditionalArmRaiseReefReady) // Use conditional trigger
      .and(l4ArmReady)
      .onTrue(rotaryPart.l4coralScore().alongWith(intake.coralHold()));

  safeButton5
      .and(coralMode)
      .and(conditionalArmReefReady) // Use conditional trigger
      .and(l4ScoreReady)
      .onTrue(Commands.waitSeconds(.5).andThen(intake.outTake()));*/

  // }

  public Command l4ScoreCommandFactory(int leftRight, int tagTo) {
    BooleanSupplier armRaisePositionCheck = () -> armFieldAutoReady(leftRight, 1, tagTo);
    Trigger conditionalRaiseReefReady = new Trigger(armRaisePositionCheck);
    BooleanSupplier armReefPositionCheck =
        () -> !autoPathingEnabled || armFieldAutoReady(leftRight, .1, tagTo);
    Trigger conditionalArmReefReady = new Trigger(armReefPositionCheck);
    return Commands.waitUntil(armRaisePositionCheck).andThen();
  }

  /*  safeButton5
      .and(coralMode)
      .and(conditionalArmRaiseReefReady) // Use conditional trigger
      .and(l4ArmReady)
      .onTrue(rotaryPart.l4coralScore().alongWith(intake.coralHold()));

  safeButton5
      .and(coralMode)
      .and(conditionalArmReefReady) // Use conditional trigger
      .and(l4ScoreReady)
      .onTrue(Commands.waitSeconds(.5).andThen(intake.outTake()));*/

  // }

  /** Sets up the kill switch for autonomous pathing */
  private void setupTestingChooser() {
    testingChooser.setDefaultOption("NOT in debug mode", false);
    testingChooser.addOption("In Debug Mode", true);
    SmartDashboard.putData("Debug Mode", testingChooser);
    new Trigger(() -> testingChooser.getSelected() != testingBoolean)
        .onTrue(
            Commands.runOnce(
                () -> {
                  testingBoolean = testingChooser.getSelected();

                  Logger.recordOutput("DebugMode/isEnabled", testingBoolean);

                  // Cancel any active pathing commands when disabled

                }));
  }

  private void setupPathingKillSwitch() {
    pathingEnabledChooser.setDefaultOption("Auto Pathing Enabled", true);
    pathingEnabledChooser.addOption("Manual Control Only", false);
    SmartDashboard.putData("Pathing Control", pathingEnabledChooser);

    new Trigger(
            () -> pathingEnabledChooser.getSelected() != autoPathingEnabled || pathfindingOverride)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pathfindingOverride) {
                    autoPathingEnabled = !pathfindingOverride;
                  } else {
                    autoPathingEnabled = pathingEnabledChooser.getSelected();
                  }
                  Logger.recordOutput("AutoPathing/isEnabled", autoPathingEnabled);

                  // Cancel any active pathing commands when disabled

                }));
  }

  @SuppressWarnings("unused")
  private void configureBindings() {
    // Create safe versions of all joystick trigger inputs that respect the ignoreJoystickInput flag
    Trigger safePovLeft = createSafeJoystickTrigger(joystick.povLeft());
    Trigger safePovRight = createSafeJoystickTrigger(joystick.povRight());
    Trigger safePovUp = createSafeJoystickTrigger(joystick.povUp());
    Trigger safePovDown = createSafeJoystickTrigger(joystick.povDown());
    Trigger safeButton1 = createSafeJoystickTrigger(joystick.button(1));
    Trigger safeButton2 = createSafeJoystickTrigger(joystick.button(2));
    Trigger safeButton3 = createSafeJoystickTrigger(joystick.button(3));
    Trigger safeButton4 = createSafeJoystickTrigger(joystick.button(4));
    Trigger safeButton5 = createSafeJoystickTrigger(joystick.button(5));
    Trigger safeButton6 = createSafeJoystickTrigger(joystick.button(6));
    Trigger safeButton7 = createSafeJoystickTrigger(joystick.button(7));
    Trigger safeButton8 = createSafeJoystickTrigger(joystick.button(8));
    Trigger safeButton9 = createSafeJoystickTrigger(joystick.button(9));
    Trigger safeButton10 = createSafeJoystickTrigger(joystick.button(10));
    Trigger safeButton11 = createSafeJoystickTrigger(joystick.button(11));
    Trigger safeButton12 = createSafeJoystickTrigger(joystick.button(12));
    Trigger safeButton13 = createSafeJoystickTrigger(joystick.button(13));
    Trigger safeButton14 = createSafeJoystickTrigger(joystick.button(14));
    Trigger safeButton15 = createSafeJoystickTrigger(joystick.button(15));
    Trigger safeButton16 = createSafeJoystickTrigger(joystick.button(16));

    // Direction selection triggers

    safePovLeft.onTrue(switchCoralDirection(0));
    safePovRight.onTrue(switchCoralDirection(1));

    safePovDown.onTrue(
        drivetrain.runOnce(
            () -> {
              visionEnabled = true;
              drivetrain.seedFieldCentric();
            }));

    safePovUp.onTrue(
        drivetrain.runOnce(
            () -> {
              visionEnabled = false;
              drivetrain.seedFieldCentric();
            }));
    /*   // Vision/field-centric heading reset triggers

    safePovDown.onTrue(resetGyroCommand(0));

    safePovUp.onTrue(resetGyroCommand(drivetrain.getState().Pose.getRotation().getRadians()));*/

    // Mode Switch
    safeButton2.onTrue(changeMode());

    // Basic triggers (not position-dependent)
    Trigger intakeColorSensorTrigger = new Trigger(() -> intake.getIntakeDistanceBool());
    Trigger algaeHeightReady = new Trigger(() -> elevator.getElevatorHeight() > 20);
    Trigger processorReady = new Trigger(() -> elevator.getElevatorHeight() > 4);
    Trigger currentIntakeSwitch = new Trigger(() -> intake.getHighCurrent());
    Trigger algaeMode = new Trigger(() -> getModeMethod());
    Trigger l2Ready = new Trigger(() -> elevator.getElevatorHeight() > 13);
    Trigger l3Ready = new Trigger(() -> elevator.getElevatorHeight() > 29);
    Trigger l4ArmReady = new Trigger(() -> elevator.getElevatorHeight() > 58);
    Trigger l4ScoreReady = new Trigger(() -> elevator.getElevatorHeight() > 61);
    Trigger coralMode = new Trigger(() -> !getModeMethod());
    Trigger normalMode = new Trigger(() -> !testingBoolean);
    // Change later! -

    // Create conditional position triggers that bypass checks when autoPathingEnabled is false
    // For reef/field position
    BooleanSupplier armReefPositionCheck =
        () -> !autoPathingEnabled || armFieldReady(leftCoral, .25);
    Trigger conditionalArmReefReady = new Trigger(armReefPositionCheck);

    BooleanSupplier armRaiseReefPositionCheck =
        () -> !autoPathingEnabled || armFieldReady(leftCoral, 1);
    Trigger conditionalArmRaiseReefReady = new Trigger(armRaiseReefPositionCheck);

    // For algae position
    BooleanSupplier armAlgaePositionCheck = () -> !autoPathingEnabled || armAlgaeReady(.1);
    Trigger conditionalArmAlgaeReady = new Trigger(armAlgaePositionCheck);

    BooleanSupplier armRaiseAlgaePositionCheck = () -> !autoPathingEnabled || armAlgaeReady(1);
    Trigger conditionalArmRaiseAlgaeReady = new Trigger(armRaiseAlgaePositionCheck);

    // For barge position
    BooleanSupplier armBargePositionCheck = () -> !autoPathingEnabled || armBargeReady(.1);
    Trigger conditionalArmBargeReady = new Trigger(armBargePositionCheck);

    BooleanSupplier armRaiseBargePositionCheck = () -> !autoPathingEnabled || armBargeReady(1);
    Trigger conditionalArmRaiseBargeReady = new Trigger(armRaiseBargePositionCheck);

    // For processor position
    BooleanSupplier armProcessorPositionCheck = () -> !autoPathingEnabled || armProcessorReady(.1);
    Trigger conditionalArmProcessorReady = new Trigger(armProcessorPositionCheck);

    // For coral intake position
    BooleanSupplier coralIntakePositionCheck = () -> !autoPathingEnabled || !coralIntakeReady();
    Trigger conditionalCoralIntakeReady = new Trigger(coralIntakePositionCheck);

    // Drivetrain default command setup
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        1, Robot.getInstance().shouldIgnoreJoystickInput())
                                    /*Math.cos(gyroMultiplier)
                                        * -getAdjustedJoystickAxis(
                                            1, Robot.getInstance().shouldIgnoreJoystickInput())
                                    - Math.sin(gyroMultiplier)
                                        * -getAdjustedJoystickAxis(
                                            0, Robot.getInstance().shouldIgnoreJoystickInput())*/ ,
                                    .05)),
                                3)
                            * MaxSpeed)
                    .withVelocityY(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        0, Robot.getInstance().shouldIgnoreJoystickInput())
                                    /*Math.sin(gyroMultiplier)
                                        * -getAdjustedJoystickAxis(
                                            1, Robot.getInstance().shouldIgnoreJoystickInput())
                                    + Math.cos(gyroMultiplier)
                                        * -getAdjustedJoystickAxis(
                                            0, Robot.getInstance().shouldIgnoreJoystickInput())*/ ,
                                    .05)),
                                3)
                            * MaxSpeed)
                    .withRotationalRate(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        2, Robot.getInstance().shouldIgnoreJoystickInput()),
                                    .05)),
                                3)
                            * MaxAngularRate)));

    // Intake stop on coral leaving
    intakeColorSensorTrigger
        .and(coralMode)
        .onTrue(elevator.toBottom().alongWith(rotaryPart.coralScore()).andThen(intake.stop()));

    // Coral intake - Button 3
    safeButton3
        .and(coralMode)
        .and(intakeColorSensorTrigger)
        // .and(conditionalCoralIntakeReady) // Use conditional trigger
        .whileTrue(intake.intake().alongWith(rotaryPart.store()).alongWith(elevator.toBottom()))
        .onFalse((rotaryPart.coralScore()).alongWith(intake.stop()));

    // Coral station pathfinding - Button 3
    safeButton3
        // .and(coralIntakePositionCheck)
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Only schedule the command if auto pathing is enabled
                  if (autoPathingEnabled) {
                    stationCommand = m_pathfinder.getPathfindingCommandStation(getClosestStation());
                    stationCommand.schedule();
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (stationCommand != null) {
                    stationCommand.cancel();
                  }
                }));
    // L2 Reef - Button 7
    safeButton7
        .and(normalMode)
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (autoPathingEnabled) {
                    reefCommand =
                        m_pathfinder.getPathfindingCommandReef(leftCoral, getClosestTag());
                    reefCommand.schedule();
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (reefCommand != null) {
                    reefCommand.cancel();
                  }
                }));

    safeButton7
        .and(normalMode)
        .and(coralMode)
        .and(conditionalArmReefReady) // Use conditional trigger
        .and(l2Ready)
        .onTrue(Commands.waitSeconds(.3).andThen(intake.outTake()));

    safeButton7
        .and(normalMode)
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .onTrue(rotaryPart.coralScore().alongWith(elevator.toL2()));

    // L3 Reef - Button 6
    safeButton6
        .and(normalMode)
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (autoPathingEnabled) {
                    reefCommand =
                        m_pathfinder.getPathfindingCommandReef(leftCoral, getClosestTag());
                    reefCommand.schedule();
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (reefCommand != null) {
                    reefCommand.cancel();
                  }
                }));

    safeButton6
        .and(normalMode)
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .and(l3Ready)
        .onTrue(Commands.waitSeconds(.3).andThen(intake.outTake()));

    safeButton6
        .and(normalMode)
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .onTrue(rotaryPart.coralScore().alongWith(elevator.toL3()));

    // L4 Reef - Button 5
    safeButton5
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (autoPathingEnabled) {
                    reefCommand =
                        m_pathfinder.getPathfindingCommandReefL4(leftCoral, getClosestTag());
                    reefCommand.schedule();
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (reefCommand != null) {
                    reefCommand.cancel();
                  }
                }));

    safeButton5
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .onTrue(rotaryPart.coralScore().alongWith(elevator.toL4()));

    safeButton5
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .and(l4ArmReady)
        .onTrue(rotaryPart.l4coralScore().alongWith(intake.coralHold()));

    safeButton5
        .and(coralMode)
        .and(conditionalArmRaiseReefReady) // Use conditional trigger
        .and(l4ScoreReady)
        .onFalse(intake.outTake());

    // Processor - Button 1
    safeButton1
        .and(algaeMode)
        .onTrue(elevator.toProcessor())
        .onFalse(
            intake
                .outTake()
                .alongWith(Commands.waitSeconds(.3))
                .andThen(
                    rotaryPart
                        .coralScore()
                        .alongWith(elevator.toBottom())
                        .alongWith(intake.stop())));

    // L2 Algae - Button 3
    /*  safeButton3
    .and(algaeMode)
    .onTrue(
        Commands.runOnce(
            () -> {
              if (autoPathingEnabled) {
                algaeCommand = m_pathfinder.getPathfindingCommandAlgae(getClosestTag());
                algaeCommand.schedule();
              }
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (algaeCommand != null) {
                algaeCommand.cancel();
              }
            }));*/

    safeButton3
        .and(algaeMode)
        // .and(conditionalArmAlgaeReady) // Use conditional trigger
        .and(algaeHeightReady)
        .onTrue(rotaryPart.algaeGrab().alongWith(intake.reverseIntake()));

    safeButton3
        .and(algaeMode)
        // .and(conditionalArmRaiseAlgaeReady) // Use conditional trigger
        .onTrue(rotaryPart.coralScore().alongWith(elevator.toL2Algae()));

    safeButton3.and(algaeMode).onFalse((intake.algaeHold()));

    // L3 Algae - Button 4
    /*safeButton4
    .and(algaeMode)
    .onTrue(
        Commands.runOnce(
            () -> {
              if (autoPathingEnabled) {
                algaeCommand = m_pathfinder.getPathfindingCommandAlgae(getClosestTag());
                algaeCommand.schedule();
              }
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (algaeCommand != null) {
                algaeCommand.cancel();
              }
            }));*/

    safeButton4
        .and(algaeMode)
        // .and(conditionalArmAlgaeReady) // Use conditional trigger
        .and(algaeHeightReady)
        .onTrue(rotaryPart.algaeGrab().alongWith(intake.reverseIntake()));

    safeButton4
        .and(algaeMode)
        // .and(conditionalArmRaiseAlgaeReady) // Use conditional trigger
        .onTrue(rotaryPart.coralScore().alongWith(elevator.toL3Algae()));

    safeButton4.and(algaeMode).onFalse((intake.algaeHold()));

    safeButton13
        .or(safeButton11)
        .or(safeButton12)
        .onTrue(
            runOnce(() -> CommandScheduler.getInstance().cancelAll())
                .alongWith(elevator.setElevatorVoltage(0))
                .alongWith(runOnce(() -> rotaryPart.setMotor(0)))
                .alongWith(intake.stop())
                .alongWith(runOnce(() -> TeleopPaused.set(true))));

    safeButton14
        .or(safeButton15)
        .or(safeButton16)
        .onTrue(runOnce(() -> pathfindingOverride = true));
    // Barge - Button 9
    safeButton9
        // .and(conditionalArmBargeReady) // Use conditional trigger
        .and(algaeMode)
        .onTrue(elevator.toL4Algae())
        .onFalse(
            rotaryPart.coralScore().alongWith(Commands.waitSeconds(.01)).andThen(intake.outTake()));

    /*safeButton9
    .and(algaeMode)
     .onTrue(
        Commands.runOnce(
            () -> {
              if (autoPathingEnabled) {
                algaeCommand =
                    m_pathfinder.getPathfindingCommandBarge(drivetrain.getState().Pose.getY());
                algaeCommand.schedule();
              }
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (algaeCommand != null) {
                algaeCommand.cancel();
              }
            }
             )); */

    // Slider control - Button
    // joystick.button(9).onTrue(switchState(true)).onFalse(switchState(false));

    // Enable telemetry
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureTestingBindings() {
    Trigger safeButton6 = createSafeJoystickTrigger(joystick.button(6));
    Trigger safeButton7 = createSafeJoystickTrigger(joystick.button(7));
    Trigger safeButton8 = createSafeJoystickTrigger(joystick.button(8));
    Trigger safeButton9 = createSafeJoystickTrigger(joystick.button(9));
    Trigger testingBindingReady = new Trigger(() -> testingBoolean);

    testingBindingReady
        .and(safeButton9)
        .onTrue(elevator.toTestingHeight())
        .onFalse(elevator.toBottom());

    testingBindingReady
        .and(safeButton8)
        .onTrue(rotaryPart.coralScore())
        .onFalse(rotaryPart.store());

    testingBindingReady.and(safeButton7).onTrue(rotaryPart.l4coralScore());

    testingBindingReady.and(safeButton6).onTrue(intake.reverseIntake());
  }

  // Switch state command
  public Command switchState(boolean bool) {
    return runOnce(() -> changeState(bool));
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
    algaeMode = !algaeMode;
  }

  public boolean getModeMethod() {
    return algaeMode;
  }

  public void resetGyro(double radians) {
    gyroMultiplier = radians;
  }

  public Command resetGyroCommand(double radians) {
    return runOnce(() -> resetGyro(radians));
  }

  public void changeCoralDirection(int i) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        if (i == 1) {
          i = 0;
        } else {
          i = 1;
        }
      }
    }
    leftCoral = i;
  }

  public Command switchCoralDirection(int i) {
    return runOnce(() -> changeCoralDirection(i));
  }

  // Reset PID controllers
  public void resetPID() {
    elevator.reset();
    rotaryPart.reset();
  }

  // Update alerts
  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(joystick.getHID().getPort()));
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    // Log the current status of automatic pathing
    Logger.recordOutput("AutoPathing/Enabled", autoPathingEnabled);
  }

  // Is this here for a reason?
  public int armLiftReady() {
    return 0;
  }

  public boolean armFieldReady(int goingLeft, double displacement) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    Pose2d targetPose = PathfindConstants.redTargetPoseReef[getClosestTag()][goingLeft];

    // If blue alliance, flip the target pose
    if (alliance == Alliance.Blue) {
      targetPose = FlipField.FieldFlip(targetPose);
    }

    double distAway =
        Math.sqrt(
            Math.pow(drivetrain.getState().Pose.getX() - targetPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - targetPose.getY(), 2));

    return distAway < displacement;
  }

  public boolean armFieldAutoReady(int goingLeft, double displacement, int apriltag) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    Pose2d targetPose = PathfindConstants.redTargetPoseReef[apriltag][goingLeft];

    // If blue alliance, flip the target pose
    if (alliance == Alliance.Blue) {
      targetPose = FlipField.FieldFlip(targetPose);
    }

    double distAway =
        Math.sqrt(
            Math.pow(drivetrain.getState().Pose.getX() - targetPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - targetPose.getY(), 2));

    return distAway < displacement;
  }

  public boolean armProcessorReady(double displacement) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    Pose2d targetPose = PathfindConstants.redTargetPoseProcessor;

    // If blue alliance, flip the target pose
    if (alliance == Alliance.Blue) {
      targetPose = FlipField.FieldFlip(targetPose);
    }

    double distAway =
        Math.sqrt(
            Math.pow(drivetrain.getState().Pose.getX() - targetPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - targetPose.getY(), 2));

    return distAway < displacement;
  }

  public boolean armAlgaeReady(double displacement) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    Pose2d targetPose = Pathfind.redAveragePoses[getClosestTag()];

    // If blue alliance, flip the target pose
    if (alliance == Alliance.Blue) {
      targetPose = FlipField.FieldFlip(targetPose);
    }

    double distAway =
        Math.sqrt(
            Math.pow(drivetrain.getState().Pose.getX() - targetPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - targetPose.getY(), 2));

    return distAway < displacement;
  }

  public boolean armBargeReady(double displacement) {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    double targetX = PathfindConstants.redTargetPoseXBarge;

    // If blue alliance, flip the X coordinate
    if (alliance == Alliance.Blue) {
      targetX = FlipField.FIELD_LENGTH_METERS - targetX;
    }

    double distAway = Math.abs(drivetrain.getState().Pose.getX() - targetX);

    return distAway < displacement;
  }

  public boolean coralIntakeReady() {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    Pose2d targetPose = PathfindConstants.redTargetPoseStation[getClosestStation()];

    // If blue alliance, flip the target pose
    if (alliance == Alliance.Blue) {
      targetPose = FlipField.FieldFlip(targetPose);
    }

    double distAway =
        Math.sqrt(
            Math.pow(drivetrain.getState().Pose.getX() - targetPose.getX(), 2)
                + Math.pow(drivetrain.getState().Pose.getY() - targetPose.getY(), 2));

    return distAway < 1;
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
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        offset = 6;
      } else if (alliance.get() == Alliance.Blue) {
        offset = 17;
      }
    } else {
      // Default behavior if alliance is unknown
      offset = 0; // or whatever default makes sense
    }
    try {
      for (int i = offset; i < (offset + 6); i++) {
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(i);
        if (tagPoseOptional.isEmpty()) {
          continue;
        }
        // Check if the Optional contains a value before calling get()
        if (tagPoseOptional.isPresent()) {
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
      }
    } catch (Exception e) {
      // Retrieve the actual exception thrown by the invoked method
      Throwable cause = e.getCause();
      System.err.println("The underlying exception was: " + cause);
      e.printStackTrace();
    }
    Logger.recordOutput("Logger/closestTagID", closestTag);
    // If no valid tag was found, return a default value (e.g., 0)
    return closestTag == -1 ? 0 : closestTag - offset;
  }

  public int getClosestStation() {
    int closestStation = -1;
    if (drivetrain.getState().Pose.getY() >= 4.025) {
      closestStation = 0;
    } else {
      closestStation = 1;
    }
    Logger.recordOutput("Logger/closestStation", closestStation);
    return closestStation;
  }

  public double getAdjustedJoystickAxis(int axis, boolean ignoreInput) {
    if (ignoreInput) {
      return 0.0;
    }
    return joystick.getRawAxis(axis);
  }

  /**
   * Checks if a button should be considered pressed, respecting the ignoreInput flag
   *
   * @param button The button index to check
   * @param ignoreInput Whether input should be ignored
   * @return true if the button is pressed and inputs are not being ignored
   */
  public boolean getAdjustedJoystickButton(int button, boolean ignoreInput) {
    if (ignoreInput) {
      return false;
    }
    return joystick.getHID().getRawButton(button);
  }

  /**
   * Gets the POV value, respecting the ignoreInput flag
   *
   * @param pov The POV index to check
   * @param ignoreInput Whether input should be ignored
   * @return POV angle in degrees, or -1 if POV is not pressed or inputs are being ignored
   */
  public int getAdjustedJoystickPOV(int pov, boolean ignoreInput) {
    if (ignoreInput) {
      return -1; // -1 means POV is not pressed
    }
    return joystick.getHID().getPOV(pov);
  }

  /**
   * Creates a trigger that considers the joystick's enabled state
   *
   * @param buttonTrigger The original button trigger
   * @return A trigger that only activates when the joystick is enabled
   */
  private Trigger createSafeJoystickTrigger(Trigger buttonTrigger) {
    return buttonTrigger.and(() -> !Robot.getInstance().shouldIgnoreJoystickInput());
  }
}

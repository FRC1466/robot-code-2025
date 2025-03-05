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
import frc.robot.constants.PathfindConstants;
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
  private Command reefCommand = null;


  @SuppressWarnings("unused")
  private Command stationCommand = null;

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

  // Autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // Subsystems
  private final Intake intake = new Intake();
  public static final RotatyPart rotatyPart = new RotatyPart();
  public static final Vision photonCamera = new Vision();
  public static final Elevator elevator = new Elevator();

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  final Pathfind m_pathfinder;

  public static boolean visionEnabled = true;

  public static int leftCoral = 0;

  // Joystick and telemetry
  public final CommandJoystick joystick = new CommandJoystick(0);
  final Telemetry logger = new Telemetry(MaxSpeed);
  @AutoLogOutput public static boolean algaeMode = false;

  public static CommandSwerveDrivetrain drivetrain;
  public static boolean sliderEnabled = false;

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
    initializeChooser();
  }

  // Initialize autonomous chooser with options
  public void initializeChooser() {
    // Create the Intake command that runs for exactly 2 seconds
    Command intakeCommand =
        intake
            .intake()
            .alongWith(rotatyPart.store())
            .alongWith(elevator.toBottom())
            .withTimeout(3) // Run for exactly 2 seconds
            .andThen(rotatyPart.coralScore().alongWith(intake.stop()));

    // L2 command - go to position, outtake, hold for 2 seconds, then return to bottom
    Command l2Command =
        elevator
            .toL2()
            .alongWith(rotatyPart.coralScore())
            .andThen(intake.outTake().withTimeout(3))
            .andThen(elevator.toBottom().alongWith(rotatyPart.coralScore()));

    // L3 command - go to position, outtake, hold for 2 seconds, then return to bottom
    Command l3Command =
        elevator
            .toL3()
            .alongWith(rotatyPart.coralScore())
            .alongWith(Commands.waitSeconds(1))
            .andThen(intake.outTake().withTimeout(1))
            .andThen(elevator.toBottom().alongWith(rotatyPart.coralScore()));

    // Create the L4 command with conditional follow-up action when elevator reaches position
    /*  Command l4Command =
    elevator
        .toL4()
        .alongWith(rotatyPart.coralScore())
        .alongWith(
            Commands.waitUntil(() -> elevator.getElevatorHeight() > 53)
                .andThen(
                    rotatyPart.l4coralScore().alongWith(intake.coralHold().withTimeout(.5)))
                .andThen(intake.outTake().withTimeout(.5))
                .andThen(elevator.toBottom().alongWith(rotatyPart.coralScore())));*/

    // Register the named commands
    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand("l2", l2Command);
    NamedCommands.registerCommand("l3", l3Command);
    //    NamedCommands.registerCommand("l4", l4Command);

    autoChooser.addOption("2 Piece", new PathPlannerAuto("2 Piece Auto"));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Piece Auto"));
    autoChooser.addOption("4 Piece", new PathPlannerAuto("4 Piece Auto Faster"));
    autoChooser.addOption("5 Piece", new PathPlannerAuto("5 Piece Auto Faster"));

    // Publish the chooser to the dashboard
    SmartDashboard.putData("Auto Selector", autoChooser.getSendableChooser());
  }

  // Configure joystick bindings
  // Declare triggers as class members
  public Trigger intakeProximityTrigger;
  public Trigger algaeHeightReady;
  public Trigger currentIntakeSwitch;
  public Trigger falseIntakeProximityTrigger;

  @SuppressWarnings("unused")
  private void configureBindings() {
    // Create safe versions of all joystick trigger inputs
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

    // Use safe triggers in all bindings
    safePovLeft
    .onTrue(switchCoralDirection(0));

         

    safePovRight
        .onTrue(switchCoralDirection(1));

    /* 
        .onTrue(
            Commands.runOnce(
                () -> {
                  rightReefCommand = m_pathfinder.getPathfindingCommandReef(1, getClosestTag());
                  rightReefCommand.schedule();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (rightReefCommand != null) {
                    rightReefCommand.cancel();
                  }
                }));*/

    /*   safeButton1 // TEMPORARY BINDING
    .onTrue(
        Commands.runOnce(
            () -> {
              stationCommand = m_pathfinder.getPathfindingCommandStation(getClosestStation());
              stationCommand.schedule();
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (stationCommand != null) {
                stationCommand.cancel();
              }
            }));*/

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        1, Robot.getInstance().shouldIgnoreJoystickInput()),
                                    .05)),
                                3)
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        0, Robot.getInstance().shouldIgnoreJoystickInput()),
                                    .05)),
                                3)
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        Math.pow(
                                (MathUtil.applyDeadband(
                                    -getAdjustedJoystickAxis(
                                        2, Robot.getInstance().shouldIgnoreJoystickInput()),
                                    .05)),
                                3)
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    Trigger intakeProximityTrigger = new Trigger(() -> intake.getIntakeDistanceBool());
    Trigger falseIntakeProximityTrigger = new Trigger(() -> !intake.getIntakeDistanceBool());
    Trigger algaeHeightReady = new Trigger(() -> elevator.getElevatorHeight() > 20);
    Trigger processorReady = new Trigger(() -> elevator.getElevatorHeight() > 4);
    Trigger currentIntakeSwitch = new Trigger(() -> intake.getHighCurrent());
    Trigger algaeMode = new Trigger(() -> getModeMethod());
    Trigger l2Ready = new Trigger(() -> elevator.getElevatorHeight() > 14);
    Trigger l3Ready = new Trigger(() -> elevator.getElevatorHeight() > 29);
    Trigger l4ArmReady = new Trigger(() -> elevator.getElevatorHeight() > 53);
    Trigger l4ScoreReady = new Trigger(() -> elevator.getElevatorHeight() > 61);
    Trigger coralMode = new Trigger(() -> !getModeMethod());
    Trigger armScoreReady = new Trigger(() -> armFieldReady(leftCoral,.1));
    Trigger armRaiseReady = new Trigger(() -> armFieldReady(leftCoral,5));
    Trigger coralIntakeReady = new Trigger(() -> coralIntakeReady());

    // reset the field-centric heading with vision on pov down and without vision on pov up
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

    // Mode Switch
    safeButton2.onTrue(changeMode());
    // Stop arm when coral leaves
    intakeProximityTrigger
        .and(coralMode)
        .onTrue(elevator.toBottom().alongWith(rotatyPart.coralScore()).andThen(intake.stop()));
    // Intake Coral
    safeButton3
        .and(coralMode)
        .and(intakeProximityTrigger)
        .and(coralIntakeReady)
        .whileTrue(intake.intake().alongWith(rotatyPart.store()).alongWith(elevator.toBottom()))
        .onFalse((rotatyPart.coralScore()).alongWith(intake.stop()));
    safeButton3
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  stationCommand = m_pathfinder.getPathfindingCommandStation(getClosestStation());
                  stationCommand.schedule();
                }))
        .onFalse(
            Commands.runOnce(
               () -> {
                 if (stationCommand != null) {
                   stationCommand.cancel();
                  }
                }));
    // L2
    safeButton7
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  reefCommand = m_pathfinder.getPathfindingCommandReef(leftCoral, getClosestTag());

              reefCommand.schedule();
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (reefCommand != null) {
                reefCommand.cancel();
              }
            }));
    safeButton7
        .and(coralMode)
        .and(armScoreReady)
        .and(l2Ready)
        .onTrue(Commands.waitSeconds(.3).andThen(intake.outTake()));
    safeButton7
        .and(coralMode)
        .and(armRaiseReady)
        .onTrue(rotatyPart.coralScore().alongWith(elevator.toL2()));  


    // L3
    safeButton6
    .and(coralMode)
    .onTrue(
        Commands.runOnce(
            () -> {
              reefCommand = m_pathfinder.getPathfindingCommandReef(leftCoral, getClosestTag());

          reefCommand.schedule();
        }))
.onFalse(
    Commands.runOnce(
        () -> {
          if (reefCommand != null) {
            reefCommand.cancel();
          }
        }));
safeButton6
    .and(coralMode)
    .and(armScoreReady)
    .and(l3Ready)
    .onTrue(Commands.waitSeconds(.3).andThen(intake.outTake()));
safeButton6
    .and(coralMode)
    .and(armRaiseReady)
    .onTrue(rotatyPart.coralScore().alongWith(elevator.toL3()));  
    
    // L4
    safeButton5
        .and(coralMode)
        .onTrue(
            Commands.runOnce(
                () -> {
                  reefCommand = m_pathfinder.getPathfindingCommandReef(leftCoral, getClosestTag());

              reefCommand.schedule();
            }))
    .onFalse(
        Commands.runOnce(
            () -> {
              if (reefCommand != null) {
                reefCommand.cancel();
              }
            }));

    //L4 begining to raise arm
    safeButton5
        .and(coralMode)
        .and(armRaiseReady)
        .onTrue(rotatyPart.coralScore().alongWith(elevator.toL4())); 
    //Flip arm over and apply a low negative current to hold
    safeButton5
        .and(coralMode)
        .and(armRaiseReady)
        .and(l4ArmReady)
        .onTrue(rotatyPart.l4coralScore().alongWith(intake.coralHold()));
    //Arm is at max height and in position to score
    safeButton5
        .and(coralMode)
        .and(armScoreReady)
        .and(l4ScoreReady)
        .onTrue(Commands.waitSeconds(.3).andThen(intake.outTake()));  
 
    // Processor
    /*  safeButton1
    .and(algaeMode)
    .and(processorReady)
    .onTrue(intake.outTake())
    .onFalse(rotatyPart.coralScore().alongWith(elevator.toBottom()));*/

    safeButton1
        .and(algaeMode)
        .onTrue(elevator.toProcessor())
        .onFalse(
            intake
                .outTake()
                .alongWith(Commands.waitSeconds(.3))
                .andThen(
                    rotatyPart
                        .coralScore()
                        .alongWith(elevator.toBottom())
                        .alongWith(intake.stop())));
    // L2 Removal
    safeButton3.and(algaeMode).onTrue(rotatyPart.coralScore().alongWith(elevator.toL2Algae()));
    safeButton3
        .and(algaeMode)
        .and(algaeHeightReady)
        .onTrue(rotatyPart.algaeGrab().alongWith(intake.reverseIntake()));
    safeButton3.and(algaeMode).and(currentIntakeSwitch).onFalse((intake.algaeHold()));
    // L3 Removal
    safeButton4.and(algaeMode).onTrue(rotatyPart.coralScore().alongWith(elevator.toL3Algae()));
    safeButton4
        .and(algaeMode)
        .and(algaeHeightReady)
        .onTrue(rotatyPart.algaeGrab().alongWith(intake.reverseIntake()));
    safeButton4.and(algaeMode).and(currentIntakeSwitch).onFalse((intake.algaeHold()));
    safeButton9
        .and(algaeMode)
        .onTrue(elevator.toL4Algae())
        .onFalse(
            rotatyPart.coralScore().alongWith(Commands.waitSeconds(.01)).andThen(intake.outTake()));

    joystick.button(12).onTrue(switchState(true)).onFalse(switchState(false));
    drivetrain.registerTelemetry(logger::telemeterize);
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

  public void changeCoralDirection(int i){
    leftCoral = i;
  }

  public Command switchCoralDirection(int i){
    return runOnce(() -> changeCoralDirection(i));
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

  public int armLiftReady() {
    return 0;
  }

  public boolean armFieldReady(int goingLeft, double displacement) {
    double distAway =
        Math.sqrt(
            Math.pow(
                    drivetrain.getState().Pose.getX()
                        - PathfindConstants.redTargetPoseReef[getClosestTag()][goingLeft].getX(),
                    2)
                + Math.pow(
                    drivetrain.getState().Pose.getY()
                        - PathfindConstants.redTargetPoseReef[getClosestTag()][goingLeft].getY(),
                    2));
    if (distAway < displacement) {
      return true;
    }
    return false;
  }

  public boolean coralIntakeReady() {
    double distAway =
        Math.sqrt(
            Math.pow(
                    drivetrain.getState().Pose.getX()
                        - PathfindConstants.redTargetPoseStation[getClosestStation()].getX(),
                    2)
                + Math.pow(
                    drivetrain.getState().Pose.getY()
                        - PathfindConstants.redTargetPoseStation[getClosestStation()].getY(),
                    2));
    if (distAway < .1) {
      return true;
    }
    return false;
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

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.RotaryPartSim;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that tracks and visualizes game pieces (coral and algae) for the 2025 REEFSCAPE game.
 * Shows both currently held pieces and pieces that have been scored on the field.
 */
public class GamePieceTracker extends SubsystemBase {
  private static GamePieceTracker instance;

  // Current state tracking
  private boolean hasAlgae = false;
  public boolean hasCoral = false;
  private boolean lastIntakeProximity = false;
  private boolean justScored = false;
  private int scoreCooldown = 0;

  // Reference to the RobotContainer instance
  private final RobotContainer m_robotContainer;

  // References to subsystems and inputs
  private final Intake m_intake;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final CommandJoystick m_joystick;
  private final Elevator m_elevator;
  private final RotaryPartSim m_rotaryPartSim;

  // Track scored game pieces with unique IDs to prevent duplicates
  private final Set<Pose3d> scoredCoralPoses = new HashSet<>();
  private final Set<Pose3d> scoredAlgaePoses = new HashSet<>();

  // Map to track game piece IDs for visualization (useful for logging/debugging)
  private final ConcurrentHashMap<String, Pose3d> gamePieceMap = new ConcurrentHashMap<>();

  // Counter for limiting rate of logging
  private int loggingCounter = 0;

  // Cache for the last read Mechanism3d component poses
  private Pose3d[] lastComponentPoses = null;

  // Tunable offsets
  private final LoggedTunableNumber algaeOffsetX = new LoggedTunableNumber("AlgaeOffsetX", .535);
  private final LoggedTunableNumber algaeOffsetY = new LoggedTunableNumber("AlgaeOffsetY", 0);
  private final LoggedTunableNumber algaeOffsetZ = new LoggedTunableNumber("AlgaeOffsetZ", -0.01);
  private final LoggedTunableNumber coralOffsetX = new LoggedTunableNumber("CoralOffsetX", 0);
  private final LoggedTunableNumber coralOffsetZ = new LoggedTunableNumber("CoralOffsetZ", .3);
  private final LoggedTunableNumber coralOffsetRotY =
      new LoggedTunableNumber("CoralOffsetRotY", 42);

  /**
   * Gets the singleton instance of the GamePieceTracker.
   *
   * @return The GamePieceTracker instance
   */
  public static GamePieceTracker getInstance() {
    if (instance == null) {
      instance = new GamePieceTracker();
    }
    return instance;
  }

  /** Private constructor for singleton pattern. */
  private GamePieceTracker() {
    // Get the RobotContainer instance
    m_robotContainer = Robot.getInstance().getRobotContainer();

    // Get references to subsystems and inputs
    m_intake = RobotContainer.intake; // This is already static
    m_drivetrain = RobotContainer.drivetrain; // This is already static
    m_joystick = m_robotContainer.joystick;
    m_elevator = RobotContainer.elevator; // This is already static
    m_rotaryPartSim = RobotContainer.rotaryPartSim; // This is already static
  }

  @Override
  public void periodic() {
    try {
      // Check for game piece state changes
      updateGamePieceState();
    } catch (Exception e) {
      e.printStackTrace();
      System.err.println("Error in updateGamePieceState: " + e.getMessage());
    }

    try {
      updateVisualization();
    } catch (Exception e) {
      e.printStackTrace();
      System.err.println("Error in updateVisualization: " + e.getMessage());
    }

    // Decrement score cooldown if active
    if (scoreCooldown > 0) {
      scoreCooldown--;
    }

    // Reset the justScored flag after a short delay
    if (justScored && scoreCooldown == 0) {
      justScored = false;
    }
  }

  /** Updates the state of the game pieces based on sensors and command state. */
  private void updateGamePieceState() {
    boolean isSimulation = Constants.getRobot() == Constants.RobotType.SIMBOT;

    // In real robot mode, check the intake proximity sensor
    if (!isSimulation) {
      // Check for coral using the intake proximity sensor
      boolean currentIntakeProximity = m_intake.getIntakeDistanceBool();

      // Detect rising edge (object entering the intake)
      if (currentIntakeProximity && !lastIntakeProximity) {
        if (!hasAlgae && m_robotContainer.getModeMethod()) {
          // In algae mode, it's an algae piece
          hasAlgae = true;
          hasCoral = false;
        } else if (!hasCoral && !m_robotContainer.getModeMethod()) {
          // In coral mode, it's a coral piece
          hasCoral = true;
          hasAlgae = false;
        }
      }

      lastIntakeProximity = currentIntakeProximity;

    } else {
      // In simulation, track button presses for creating game pieces
      // Button 3 in coral mode creates coral
      if (m_joystick.button(3).getAsBoolean()
          && !m_robotContainer.getModeMethod()
          && m_robotContainer.coralIntakeReady()) {
        hasCoral = true;
        hasAlgae = false;
      }

      // Button 3 and 4 in algae mode creates algae
      if ((m_joystick.button(3).getAsBoolean() || m_joystick.button(4).getAsBoolean())
          && m_robotContainer.getModeMethod()
          && m_robotContainer.armAlgaeReady(.5)) {
        hasAlgae = true;
        hasCoral = false;
      }
    }

    // Check for scoring actions - buttons 5, 6, 7 in coral mode with coral position ready
    if (hasCoral && !justScored) {
      if (m_joystick.button(5).getAsBoolean()
          && !m_robotContainer.getModeMethod()
          && m_robotContainer.armFieldReady(RobotContainer.leftCoral, 0.25)
          && m_elevator.getElevatorHeight() > 58) { // Check if elevator is high enough for L4
        // L4 Reef scoring
        scoreCoral(m_robotContainer.getClosestTag(), ReefLevel.L4);
        setJustScored();
      } else if (m_joystick.button(6).getAsBoolean()
          && !m_robotContainer.getModeMethod()
          && m_robotContainer.armFieldReady(RobotContainer.leftCoral, 0.25)
          && m_elevator.getElevatorHeight() > 29) { // Check if elevator is high enough for L3
        // L3 Reef scoring
        scoreCoral(m_robotContainer.getClosestTag(), ReefLevel.L3);
        setJustScored();
      } else if (m_joystick.button(7).getAsBoolean()
          && !m_robotContainer.getModeMethod()
          && m_robotContainer.armFieldReady(RobotContainer.leftCoral, 0.25)
          && m_elevator.getElevatorHeight() > 13) { // Check if elevator is high enough for L2
        // L2 Reef scoring
        scoreCoral(m_robotContainer.getClosestTag(), ReefLevel.L2);
        setJustScored();
      }
    }

    // Check for algae scoring actions - buttons 1, 9 in algae mode with position ready
    if (hasAlgae && !justScored) {
      if (!m_joystick.button(1).getAsBoolean()
          && m_robotContainer.getModeMethod()
          && m_robotContainer.armProcessorReady(0.25)) {
        // Processor scoring
        scoreAlgae(0); // Position 0 for processor
      } else if (!m_joystick.button(9).getAsBoolean()
          && m_robotContainer.getModeMethod()
          && m_robotContainer.armBargeReady(0.25)) {
        // Barge scoring
        scoreAlgae(0);
      }
    }

    // Detect if the outTake button (5 for L4, 6 for L3, 7 for L2) is released in coral mode
    // This helps ensure we detect when the coral is actually ejected from the robot
    if (hasCoral) {
      boolean outTakeButton = false;
      if (!m_robotContainer.getModeMethod()) { // Coral mode
        outTakeButton =
            (m_joystick.button(5).getAsBoolean() && m_elevator.getElevatorHeight() > 61)
                || // L4 score ready
                (m_joystick.button(6).getAsBoolean() && m_elevator.getElevatorHeight() > 29)
                || // L3 score ready
                (m_joystick.button(7).getAsBoolean()
                    && m_elevator.getElevatorHeight() > 13); // L2 score ready

        // If outTake is triggered and we're in position to score, mark the coral as scored
        if (outTakeButton && m_robotContainer.armFieldReady(RobotContainer.leftCoral, 0.25)) {
          // We'll score the coral at the next position check
        }
      }
    }
  }

  /** Sets the just scored flag and initializes the cooldown to prevent multiple scores. */
  private void setJustScored() {
    justScored = true;
    scoreCooldown = 20; // About 0.4 seconds at 50Hz
  }

  /** Updates the 3D visualization of all game pieces. */
  private void updateVisualization() {
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      return;
    }

    // Get the latest mechanism component poses
    updateComponentPoses();

    // Visualize held game pieces
    visualizeHeldGamePieces();

    // Visualize scored game pieces
    visualizeScoredGamePieces();
  }

  /**
   * Calculates the mechanism component poses directly, similar to how MechanismVisualizer does it.
   */
  private void updateComponentPoses() {
    try {
      // Get the elevator height in meters
      double elevatorHeightMeters = m_elevator.visualizationMeters;

      // Constants for component calculations (matching the ones in MechanismVisualizer)
      final double comp1MaxHeight = 0.925;
      final double comp2HeightMultiplier = 0.5;
      final double comp3XOffset = 0.28;
      final double comp3ZOffset = 0.365;
      final double comp3YOffset = 0;
      final double comp3YRotOffset = -30;
      final double armAngleMultiplier = -1;

      // Calculate height ratio
      double totalMaxHeight = comp1MaxHeight + comp2HeightMultiplier;
      double heightRatio = elevatorHeightMeters / totalMaxHeight;

      // Calculate component heights
      double comp1Height = Math.min(heightRatio * comp1MaxHeight, comp1MaxHeight);
      double comp2Height = comp1Height + (heightRatio * comp2HeightMultiplier);

      // Calculate arm angle
      double armAngleDegrees =
          armAngleMultiplier * Units.radiansToDegrees(m_rotaryPartSim.getAngleForVisualization());

      // Create the component poses
      Pose3d component1Pose = new Pose3d(new Translation3d(0, 0, comp1Height), new Rotation3d());
      Pose3d component2Pose = new Pose3d(new Translation3d(0, 0, comp2Height), new Rotation3d());

      // Third component - The arm (with rotation)
      Pose3d component3Pose =
          new Pose3d(
              new Translation3d(comp3XOffset, comp3YOffset, comp2Height + comp3ZOffset),
              new Rotation3d(
                  0,
                  Units.degreesToRadians(armAngleDegrees) + Units.degreesToRadians(comp3YRotOffset),
                  0));

      // Store the calculated poses
      lastComponentPoses = new Pose3d[] {component1Pose, component2Pose, component3Pose};
    } catch (Exception e) {
      // If anything goes wrong, don't update the poses
      System.err.println("Error calculating component poses: " + e.getMessage());
    }
  }

  /** Creates a visualization of the game pieces currently held by the robot. */
  private void visualizeHeldGamePieces() {
    // For algae visualization, also check the intake proximity sensor as in RobotContainer
    if (hasAlgae && m_intake.getIntakeDistanceBool()) {
      Pose3d robotPose = new Pose3d(m_drivetrain.getState().Pose);
      Pose3d armPose = lastComponentPoses[2];
      Logger.recordOutput(
          "Mechanism3d/GamePieces/Algae",
          robotPose.transformBy(
              new Transform3d(
                  new Translation3d(
                      algaeOffsetX.getAsDouble(),
                      armPose.getY() + algaeOffsetY.getAsDouble(),
                      armPose.getZ() + algaeOffsetZ.getAsDouble()),
                  Rotation3d.kZero)));
    } else {
      // If no algae or proximity condition not met, clear the visualization
      Logger.recordOutput("Mechanism3d/GamePieces/Algae", new Pose3d());
    }

    // For coral visualization, also require the robot to be in a good intake position
    if (hasCoral && lastComponentPoses != null && lastComponentPoses.length >= 3) {
      Pose3d robotPose = new Pose3d(m_drivetrain.getState().Pose);
      Pose3d armPose = lastComponentPoses[2];
      double armAngle = armPose.getRotation().getY();
      // Create the coral pose at the end of the arm
      Pose3d coralPoseRelative =
          new Pose3d(
              armPose.getX() + coralOffsetX.getAsDouble(),
              armPose.getY(),
              armPose.getZ() + coralOffsetZ.getAsDouble(),
              new Rotation3d(
                  0, armAngle + Units.degreesToRadians(coralOffsetRotY.getAsDouble()), 0));

      Logger.recordOutput(
          "Mechanism3d/GamePieces/Coral",
          robotPose.transformBy(new Transform3d(Pose3d.kZero, coralPoseRelative)));
    } else {
      // If no coral or intake condition not met, clear the visualization
      Logger.recordOutput("Mechanism3d/GamePieces/Coral", new Pose3d());
    }
  }

  /** Visualizes all scored game pieces on the field. */
  private void visualizeScoredGamePieces() {
    // Record coral positions as an array of Pose3d objects
    if (!scoredCoralPoses.isEmpty()) {
      Logger.recordOutput("ObjectiveTracker/3DView/Coral", scoredCoralPoses.toArray(Pose3d[]::new));
    }

    // Record algae positions as an array of Pose3d objects (was previously Translation3d)
    if (!scoredAlgaePoses.isEmpty()) {
      Logger.recordOutput("ObjectiveTracker/3DView/Algae", scoredAlgaePoses.toArray(Pose3d[]::new));
    }
  }

  /**
   * Records coral as scored on the reef at the specified branch and level.
   *
   * @param branchId The branch ID (0-5)
   * @param level The reef level (L1-L4)
   */
  public void scoreCoral(int branchId, ReefLevel level) {
    if (hasCoral) {
      // Calculate the position for coral
      Pose3d coralPose = calculateCoralPose((branchId + 5) % 6, level);

      // Add to the set of scored coral
      scoredCoralPoses.add(coralPose);

      // Add to map with unique ID for tracking
      String pieceId = "coral_" + UUID.randomUUID().toString().substring(0, 8);
      gamePieceMap.put(pieceId, coralPose);

      // Reset the held state
      hasCoral = false;

      Logger.recordOutput("GamePieces/ScoredCoral", scoredCoralPoses.size());
    }
  }

  /**
   * Records algae as scored on the field at the specified section.
   *
   * @param section The section number (0-5)
   */
  public void scoreAlgae(int section) {
    if (hasAlgae) {
      // Calculate the position for algae
      Pose3d algaePosition = calculateAlgaePosition(section);

      // Add to the set of scored algae
      scoredAlgaePoses.add(algaePosition);

      // Reset the held state
      hasAlgae = false;

      Logger.recordOutput("GamePieces/ScoredAlgae", scoredAlgaePoses.size());
    }
  }

  /**
   * Gets whether the robot is currently holding algae.
   *
   * @return True if the robot has algae, false otherwise
   */
  public boolean hasAlgae() {
    return hasAlgae;
  }

  /**
   * Gets whether the robot is currently holding coral.
   *
   * @return True if the robot has coral, false otherwise
   */
  public boolean hasCoral() {
    return hasCoral;
  }

  /**
   * Manually sets whether the robot has algae.
   *
   * @param hasAlgae Whether the robot should have algae
   */
  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
    if (hasAlgae) {
      this.hasCoral = false; // Can't have both at once
    }
  }

  /**
   * Manually sets whether the robot has coral.
   *
   * @param hasCoral Whether the robot should have coral
   */
  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
    if (hasCoral) {
      this.hasAlgae = false; // Can't have both at once
    }
  }

  /**
   * Calculates the 3D pose for coral being scored on a reef branch.
   *
   * @param branchId The branch ID (0-5)
   * @param level The reef level
   * @return A Pose3d for visualization
   */
  private Pose3d calculateCoralPose(int branchId, ReefLevel level) {
    // Get the current alliance
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    // Calculate the branch index in the branchPositions array
    // branchId (0-5) represents the section of the reef
    // Each section has two branches (left and right)
    int section = branchId;
    int leftOrRight;

    // Determine if we should use the left or right branch in this section
    if (alliance == Alliance.Blue) {
      // For Blue alliance:
      // leftCoral=0 means we want the left branch (index 0)
      // leftCoral=1 means we want the right branch (index 1)
      leftOrRight = RobotContainer.leftCoral;
    } else {
      // For Red alliance:
      // leftCoral=0 means we want the right branch (index 1)
      // leftCoral=1 means we want the left branch (index 0)
      leftOrRight = 1 - RobotContainer.leftCoral;
    }

    // Calculate the final index in the branchPositions array
    int branchIndex = section * 2 + leftOrRight;

    // Get the branch pose from field constants
    Pose3d branchPose = FieldConstants.Reef.branchPositions.get(branchIndex).get(level);

    // Adjust position to properly visualize coral
    Pose3d adjustedPose =
        branchPose.transformBy(
            new Transform3d(
                new Translation3d(-Units.inchesToMeters(11.875) / 2.5, 0.0, 0.0),
                Rotation3d.kZero));

    // Only flip if necessary for the current alliance
    if (alliance == Alliance.Red) {
      return FlipField.flipIfRed(adjustedPose);
    }

    return adjustedPose;
  }

  /**
   * Calculates the 3D position for algae being scored.
   *
   * @param section The section number (0-5)
   * @return A Translation3d for visualization
   */
  private Pose3d calculateAlgaePosition(int section) {
    // Get current alliance
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = allianceOptional.orElse(Alliance.Blue);

    // Get the branch positions for this section
    section = Math.min(Math.max(section, 0), 5); // Clamp to valid range

    var firstBranchPose = FieldConstants.Reef.branchPositions.get(section * 2).get(ReefLevel.L2);
    var secondBranchPose =
        FieldConstants.Reef.branchPositions.get(section * 2 + 1).get(ReefLevel.L3);

    // Calculate the position for algae (interpolate between two branches)
    Pose3d algaePosition =
        new Pose3d(
            firstBranchPose
                .getTranslation()
                .interpolate(secondBranchPose.getTranslation(), 0.5)
                .plus(
                    new Translation3d(
                        -FieldConstants.algaeDiameter / 3.0,
                        0.0,
                        (section % 2 == 0)
                            ? secondBranchPose.getZ() - firstBranchPose.getZ()
                            : 0.0)),
            new Rotation3d(0.0, -35.0 / 180.0 * Math.PI, firstBranchPose.getRotation().getZ()));

    // Only flip if on red alliance
    if (alliance == Alliance.Red) {
      algaePosition = FlipField.flipIfRed(algaePosition);
    }

    return algaePosition;
  }

  /** Clears all tracked game pieces. */
  public void clearGamePieces() {
    scoredCoralPoses.clear();
    scoredAlgaePoses.clear();
    gamePieceMap.clear();
    hasAlgae = false;
    hasCoral = false;
    justScored = false;
    scoreCooldown = 0;
  }
}

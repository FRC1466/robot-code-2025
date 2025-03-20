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
  private boolean hasL2Algae = false; // Specifically for L2 algae
  private boolean hasL3Algae = false; // Specifically for L3 algae
  public boolean hasCoral = false;
  private boolean lastIntakeProximity = false;
  private boolean justScored = false;
  private int scoreCooldown = 0;
  private boolean justIntaked = false; // New flag for intake cooldown
  private int intakeCooldown = 0; // New cooldown for intake actions

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

    // Initialize field with algae in alternating heights
    initializeFieldWithAlgae();
  }

  /**
   * Initializes the field with algae at alternating heights. This places algae at L2 and L3
   * positions alternating around the reef.
   */
  private void initializeFieldWithAlgae() {
    // Create algae at all 6 positions around the reef, exactly matching the FRC 6328 implementation
    for (int i = 0; i < 6; i++) {
      var firstBranchPose = FieldConstants.Reef.branchPositions.get(i * 2).get(ReefLevel.L2);
      var secondBranchPose = FieldConstants.Reef.branchPositions.get(i * 2 + 1).get(ReefLevel.L3);

      Translation3d algaePosition = new Translation3d();
      algaePosition =
          AllianceFlipUtil.apply(
              firstBranchPose
                  .getTranslation()
                  .interpolate(secondBranchPose.getTranslation(), 0.5)
                  .plus(
                      new Translation3d(
                          -FieldConstants.algaeDiameter / 3.0,
                          new Rotation3d(
                              0.0, -35.0 / 180.0 * Math.PI, firstBranchPose.getRotation().getZ())))
                  .plus(
                      new Translation3d(
                          0.0,
                          0.0,
                          (i % 2 == 0) ? secondBranchPose.getZ() - firstBranchPose.getZ() : 0.0)));

      // Create a new Pose3d for each algae with a unique object reference
      Pose3d algaePose = new Pose3d(algaePosition, new Rotation3d());

      // Generate unique ID for tracking
      boolean isL3 = (i % 2 == 0); // Even positions are L3, odd are L2
      String pieceId =
          "initial_algae_"
              + i
              + "_"
              + (isL3 ? "L3" : "L2")
              + "_"
              + UUID.randomUUID().toString().substring(0, 4);

      // Add to the map first, then to the set
      gamePieceMap.put(pieceId, algaePose);
      scoredAlgaePoses.add(algaePose);

      Logger.recordOutput("GamePieces/InitializedPiece", pieceId);
    }
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

    // Decrement intake cooldown if active
    if (intakeCooldown > 0) {
      intakeCooldown--;
    }

    // Reset the justIntaked flag after a short delay
    if (justIntaked && intakeCooldown == 0) {
      justIntaked = false;
    }
  }

  /** Updates the state of the game pieces based on sensors and command state. */
  private void updateGamePieceState() {
    boolean isSimulation = Constants.getRobot() == Constants.RobotType.SIMBOT;

    // In real robot mode, check the intake proximity sensor
    if (!isSimulation) {
      // Check for coral using the intake proximity sensor
      boolean currentIntakeProximity = m_intake.getIntakeDistanceBool();

      // Detect rising edge (object entering the intake) and make sure we're not in cooldown
      if (currentIntakeProximity && !lastIntakeProximity && !justIntaked) {
        if (!hasL2Algae
            && !hasL3Algae
            && m_robotContainer.getModeMethod()
            && m_joystick.button(3).getAsBoolean()) {
          // Button 3 in algae mode is for L2 algae
          hasL2Algae = true;
          hasL3Algae = false;
          hasAlgae = true;
          hasCoral = false;

          // Remove the L2 algae from field at the closest position
          removeAlgaeFromField(true); // true = L2 algae
          setJustIntaked(); // Prevent multiple intakes in quick succession
        } else if (!hasL2Algae
            && !hasL3Algae
            && m_robotContainer.getModeMethod()
            && m_joystick.button(4).getAsBoolean()) {
          // Button 4 in algae mode is for L3 algae
          hasL3Algae = true;
          hasL2Algae = false;
          hasAlgae = true;
          hasCoral = false;

          // Remove the L3 algae from field at the closest position
          removeAlgaeFromField(false); // false = L3 algae
          setJustIntaked(); // Prevent multiple intakes in quick succession
        } else if (!hasCoral && !m_robotContainer.getModeMethod()) {
          // In coral mode, it's a coral piece
          hasCoral = true;
          hasAlgae = false;
          hasL2Algae = false;
          hasL3Algae = false;
          setJustIntaked(); // Prevent multiple intakes in quick succession
        }
      }

      lastIntakeProximity = currentIntakeProximity;

    } else {
      // In simulation, track button presses for creating game pieces
      // Button 3 in coral mode creates coral - but only if not in cooldown
      if (m_joystick.button(3).getAsBoolean()
          && !m_robotContainer.getModeMethod()
          && m_robotContainer.coralIntakeReady()
          && !justIntaked) {
        hasCoral = true;
        hasAlgae = false;
        hasL2Algae = false;
        hasL3Algae = false;
        setJustIntaked(); // Set cooldown
      }

      // Button 3 in algae mode creates L2 algae - but only if not in cooldown
      if (m_joystick.button(3).getAsBoolean()
          && m_robotContainer.getModeMethod()
          && m_robotContainer.armAlgaeReady(.5)
          && !justIntaked) {
        hasL2Algae = true;
        hasL3Algae = false;
        hasAlgae = true;
        hasCoral = false;

        // Remove the L2 algae from field at the closest position
        removeAlgaeFromField(true); // true = L2 algae
        setJustIntaked(); // Set cooldown
      }

      // Button 4 in algae mode creates L3 algae - but only if not in cooldown
      if (m_joystick.button(4).getAsBoolean()
          && m_robotContainer.getModeMethod()
          && m_robotContainer.armAlgaeReady(.5)
          && !justIntaked) {
        hasL3Algae = true;
        hasL2Algae = false;
        hasAlgae = true;
        hasCoral = false;

        // Remove the L3 algae from field at the closest position
        removeAlgaeFromField(false); // false = L3 algae
        setJustIntaked(); // Set cooldown
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

  /**
   * Removes the closest algae from the field based on the type (L2 or L3)
   *
   * @param isL2 true for L2 algae, false for L3 algae
   */
  private void removeAlgaeFromField(boolean isL2) {
    try {
      // Instead of finding the closest algae directly, use the closest tag
      int closestTag = m_robotContainer.getClosestTag();
      Logger.recordOutput("GamePieces/ClosestTag", closestTag);

      // Debug the tag to section mapping
      int rawSection = (closestTag - 1) % 6;

      // Fix for 2025 reef numbering - translate the April tag IDs properly
      // The mapping may need to be adjusted based on your field's actual tag layout
      int reefSection;

      // Map tags to sections - you may need to adjust this based on your specific field setup
      switch (closestTag) {
        case 1:
          reefSection = 0;
          break;
        case 2:
          reefSection = 1;
          break;
        case 3:
          reefSection = 2;
          break;
        case 4:
          reefSection = 3;
          break;
        case 5:
          reefSection = 4;
          break;
        case 6:
          reefSection = 5;
          break;
        case 7:
          reefSection = 0;
          break;
        case 8:
          reefSection = 1;
          break;
        case 9:
          reefSection = 2;
          break;
        case 10:
          reefSection = 3;
          break;
        case 11:
          reefSection = 4;
          break;
        case 12:
          reefSection = 5;
          break;
        default:
          reefSection = 0;
          break; // Default to section 0
      }

      Logger.recordOutput("GamePieces/RawSection", rawSection);
      Logger.recordOutput("GamePieces/TargetReefSection", reefSection);

      // For debugging, print the list of all algae by section
      StringBuilder bySection = new StringBuilder();
      for (int i = 0; i < 6; i++) {
        bySection.append("Section ").append(i).append(": ");
        int l2Count = 0, l3Count = 0;
        for (String id : gamePieceMap.keySet()) {
          if (!id.startsWith("initial_algae_")) continue;
          try {
            int sectionNum =
                Integer.parseInt(
                    id.substring(
                        "initial_algae_".length(), id.indexOf('_', "initial_algae_".length())));
            if (sectionNum == i) {
              if (id.contains("_L2_")) l2Count++;
              if (id.contains("_L3_")) l3Count++;
            }
          } catch (Exception e) {
            continue;
          }
        }
        bySection.append("L2=").append(l2Count).append(", L3=").append(l3Count).append(" | ");
      }
      Logger.recordOutput("GamePieces/AlgaeBySection", bySection.toString());

      // Find the algae on this section that matches our type
      String targetAlgaeId = null;
      Pose3d targetAlgaePose = null;

      // Look for algae in that section and of the right type
      for (String pieceId : gamePieceMap.keySet()) {
        if (!pieceId.startsWith("initial_algae_")) continue;

        try {
          int underscoreIndex = pieceId.indexOf('_', "initial_algae_".length());
          String sectionStr = pieceId.substring("initial_algae_".length(), underscoreIndex);
          int algaeSection = Integer.parseInt(sectionStr);

          // Specifically check for L2 or L3 properly
          boolean isL2Algae = pieceId.contains("_L2_");
          boolean isL3Algae = pieceId.contains("_L3_");

          Logger.recordOutput(
              "GamePieces/CheckingAlgae",
              pieceId
                  + ", section="
                  + algaeSection
                  + ", reefSection="
                  + reefSection
                  + ", isL2="
                  + isL2Algae
                  + ", isL3="
                  + isL3Algae
                  + ", wantL2="
                  + isL2);

          // Make sure it's the right section and right type
          if (algaeSection == reefSection && ((isL2 && isL2Algae) || (!isL2 && isL3Algae))) {
            targetAlgaeId = pieceId;
            targetAlgaePose = gamePieceMap.get(pieceId);
            break;
          }
        } catch (Exception e) {
          // In case of parsing error, skip this piece
          continue;
        }
      }

      // If no algae found in exact section, try looking in adjacent sections
      if (targetAlgaeId == null) {
        int[] adjacentSections = {
          (reefSection + 5) % 6, // left section
          (reefSection + 1) % 6 // right section
        };

        Logger.recordOutput(
            "GamePieces/CheckingAdjacentSections",
            "Left=" + adjacentSections[0] + ", Right=" + adjacentSections[1]);

        for (int adjSection : adjacentSections) {
          for (String pieceId : gamePieceMap.keySet()) {
            if (!pieceId.startsWith("initial_algae_")) continue;

            try {
              int underscoreIndex = pieceId.indexOf('_', "initial_algae_".length());
              String sectionStr = pieceId.substring("initial_algae_".length(), underscoreIndex);
              int algaeSection = Integer.parseInt(sectionStr);

              boolean isL2Algae = pieceId.contains("_L2_");
              boolean isL3Algae = pieceId.contains("_L3_");

              if (algaeSection == adjSection && ((isL2 && isL2Algae) || (!isL2 && isL3Algae))) {
                targetAlgaeId = pieceId;
                targetAlgaePose = gamePieceMap.get(pieceId);
                Logger.recordOutput(
                    "GamePieces/FoundInAdjacent", "Found in adjacent section " + adjSection);
                break;
              }
            } catch (Exception e) {
              continue;
            }
          }
          if (targetAlgaeId != null) break; // Stop if we found one in an adjacent section
        }
      }

      // Remove the target algae if found
      if (targetAlgaeId != null && targetAlgaePose != null) {
        scoredAlgaePoses.remove(targetAlgaePose);
        gamePieceMap.remove(targetAlgaeId);
        Logger.recordOutput(
            "GamePieces/RemovedAlgae",
            "Removed " + (isL2 ? "L2" : "L3") + " Algae: " + targetAlgaeId);
      } else {
        // If still not found, search for any algae of the right type regardless of section
        Logger.recordOutput(
            "GamePieces/RemovedAlgae",
            "Could not find "
                + (isL2 ? "L2" : "L3")
                + " algae in section "
                + reefSection
                + " or adjacent sections. Searching globally...");

        for (String pieceId : gamePieceMap.keySet()) {
          if (!pieceId.startsWith("initial_algae_")) continue;

          boolean matchesType = isL2 ? pieceId.contains("_L2_") : pieceId.contains("_L3_");
          if (matchesType) {
            Pose3d algaePose = gamePieceMap.get(pieceId);
            scoredAlgaePoses.remove(algaePose);
            gamePieceMap.remove(pieceId);
            Logger.recordOutput(
                "GamePieces/RemovedAlgae",
                "Removed " + (isL2 ? "L2" : "L3") + " Algae: " + pieceId + " (global search)");
            break;
          }
        }
      }
    } catch (Exception e) {
      System.err.println("Error removing algae from field: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /** Sets the just scored flag and initializes the cooldown to prevent multiple scores. */
  private void setJustScored() {
    justScored = true;
    scoreCooldown = 20; // About 0.4 seconds at 50Hz
  }

  /** Sets the just intaked flag and initializes the cooldown to prevent multiple intakes. */
  private void setJustIntaked() {
    justIntaked = true;
    intakeCooldown = 250; // About 5 seconds at 50Hz
    Logger.recordOutput("GamePieces/IntakeCooldown", "Started");
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
      hasL2Algae = false;
      hasL3Algae = false;

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
   * @param isL2 Whether it's L2 algae (true) or L3 algae (false)
   */
  public void setHasAlgae(boolean hasAlgae, boolean isL2) {
    this.hasAlgae = hasAlgae;
    if (hasAlgae) {
      this.hasL2Algae = isL2;
      this.hasL3Algae = !isL2;
      this.hasCoral = false; // Can't have both at once
    } else {
      this.hasL2Algae = false;
      this.hasL3Algae = false;
    }
  }

  /**
   * Gets whether the robot is holding L2 algae specifically.
   *
   * @return True if the robot has L2 algae
   */
  public boolean hasL2Algae() {
    return hasL2Algae;
  }

  /**
   * Gets whether the robot is holding L3 algae specifically.
   *
   * @return True if the robot has L3 algae
   */
  public boolean hasL3Algae() {
    return hasL3Algae;
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
    return new Pose3d();
  }

  /** Clears all tracked game pieces. */
  public void clearGamePieces() {
    scoredCoralPoses.clear();
    scoredAlgaePoses.clear();
    gamePieceMap.clear();
    hasAlgae = false;
    hasL2Algae = false;
    hasL3Algae = false;
    hasCoral = false;
    justScored = false;
    scoreCooldown = 0;
    justIntaked = false;
    intakeCooldown = 0;

    // Re-initialize the field with algae in alternating heights
    initializeFieldWithAlgae();
  }

  /**
   * Resets the game piece state but keeps the initial algae on the field. Used when transitioning
   * between modes without clearing the field.
   */
  public void resetGamePieceState() {
    hasAlgae = false;
    hasL2Algae = false;
    hasL3Algae = false;
    hasCoral = false;
    justScored = false;
    scoreCooldown = 0;
    justIntaked = false;
    intakeCooldown = 0;
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * ChirpPlayer is a utility class that plays chirp files on the motors of the swerve drivetrain. It
 * uses the Orchestra class from the CTRE Phoenix library to play the chirp files.
 */
public class ChirpPlayer {

  private final AudioConfigs audioConfigs;
  public final Orchestra orchestra = new Orchestra();

  public ChirpPlayer() {
    // Configure to allow music during disabled state
    audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
    try (Orchestra orchestra = new Orchestra()) {
      // Add all drive and steer motors to the orchestra and configure audio
      java.util.Arrays.stream(CommandSwerveDrivetrain.getInstance().getModules())
          .forEach(
              module -> {
                TalonFX driveMotor = module.getDriveMotor();
                TalonFX steerMotor = module.getSteerMotor();

                // Apply audio configs to each motor
                driveMotor.getConfigurator().apply(audioConfigs);
                steerMotor.getConfigurator().apply(audioConfigs);

                // Add motors to orchestra
                orchestra.addInstrument(driveMotor);
              });
    } catch (Exception e) {
      Logger.recordOutput("Chirp Player", "Error adding motors to orchestra: " + e.getMessage());
    }
  }

  /**
   * Plays a chirp file on all motors of the swerve drivetrain.
   *
   * @param track The name of the chirp file to play (without the .chrp extension)
   * @apiNote ChirpPlayer.playChirpForAllMotors("track");
   */
  public void playChirpForAllMotors(String track) {

    // Attempt to load the chirp file
    var status = orchestra.loadMusic(track + ".chrp");

    if (!status.isOK()) {
      Logger.recordOutput("Chirp Player", "Unknown Error loading chirp file");
    } else {
      orchestra.play();
    }
  }
}

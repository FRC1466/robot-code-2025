// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.Logger;

public class Blinkin {

  Spark blinkin;

  public Blinkin() {
    blinkin = new Spark(8);
  }

  public void coralLights() {
    blinkin.set(.89);
    Logger.recordOutput("Lights", "Coral");
  }

  public void algaeLights() {
    blinkin.set(.73);
    Logger.recordOutput("Lights", "Algae");
  }

  public void rainbowPartyLights() {
    blinkin.set(-.97);
    Logger.recordOutput("Lights", "Auto");
  }

  public void warningLights() {
    blinkin.set(-0.17);
    Logger.recordOutput("Lights", "Warning");
  }
}

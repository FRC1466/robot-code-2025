// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.Logger;

public class Blinkin {

  Spark blinkin;

  public Blinkin() {
    blinkin = new Spark(8);
  }

  public void coralLights() {
    blinkin.set(.93);
    Logger.recordOutput("Lights", "Coral");
  }

  public void algaeLights() {
    blinkin.set(.77);
    Logger.recordOutput("Lights", "Algae");
  }

  public void rainbowPartyLights() {
    blinkin.set(-.97);
    Logger.recordOutput("Lights", "Auto");
  }

  public void warningLights() {
    blinkin.set(-0.11);
    Logger.recordOutput("Lights", "Warning");
  }
}

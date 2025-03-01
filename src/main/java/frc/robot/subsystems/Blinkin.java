// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

  Spark blinkin;

  public Blinkin() {

    blinkin = new Spark(1);
  }

  public void coralLights() {
    blinkin.set(.89);
  }

  public void algaeLights() {
    blinkin.set(.73);
  }

  public void rainbowPartyLights() {
    blinkin.set(-.97);
  }

  public void warningLights() {
    blinkin.set(-0.17);
  }
}

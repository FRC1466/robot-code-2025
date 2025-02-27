// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

  Spark blinkin;

  public Blinkin() {
    blinkin = new Spark(9);
  }

  /** if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93) */
  public void lightsNormal() {
    blinkin.set(0.93);
  }
}

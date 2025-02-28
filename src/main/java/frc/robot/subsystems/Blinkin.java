// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.AutoLogOutput;

public class Blinkin {

  Spark blinkin;
  @AutoLogOutput private double blinkinValue;

  public Blinkin() {
    blinkin = new Spark(0);
    blinkin.set(-.95);
    blinkinValue = -.95;
  }

  public void lightsCoral() { // pink in teleop
    blinkin.set(0.57);
    blinkinValue = .57;
  }

  public void lightsAlgae() { // green in teleop
    blinkin.set(0.77);
    blinkinValue = .77;
  }

  public void lightsAuto() { // auto should be rainbow party
    blinkin.set(-.97);
    blinkinValue = -.97;
  }

  public void lightsWarning() { // red in teleop
    blinkin.set(-.11);
    blinkinValue = -.11;
  }
}

// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
<<<<<<< Updated upstream
import org.littletonrobotics.junction.AutoLogOutput;
=======
>>>>>>> Stashed changes

public class Blinkin {

  Spark blinkin;
<<<<<<< Updated upstream
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
=======

  public Blinkin() {

    blinkin = new Spark(0);
  }

  /** if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93) */
  public void lightsNormal() {

    blinkin.set(0.93);
  }

  public void coralLights() {
    blinkin.set(.57);
  }

  public void alageLights() {
    blinkin.set(.73);
  }

  public void rainbowParty() {
    blinkin.set(-.97);
>>>>>>> Stashed changes
  }
}

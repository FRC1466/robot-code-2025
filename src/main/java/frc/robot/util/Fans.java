// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Fans {
  Spark noctuah1;
  Spark noctuah2;

  public Fans() {
    noctuah1 = new Spark(0);
    noctuah1 = new Spark(3);

    noctuah1.set(1);
    noctuah2.set(1);
  }

  public void highSpeedFans() {
    noctuah1.set(1);
    noctuah2.set(1);
  }

  public void medSpeedFans() {
    noctuah1.set(.75);
    noctuah2.set(.75);
  }

  public void lowSpeedFans() {
    noctuah1.set(.5);
    noctuah2.set(.5);
  }

  public void lowestSpeedFans() {
    noctuah1.set(.25);
    noctuah2.set(.25);
  }
}

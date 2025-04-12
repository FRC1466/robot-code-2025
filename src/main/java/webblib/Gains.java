// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package webblib;

public class Gains {
  public double P;
  public double I;
  public double D;
  public double F;
  public double integralZone;
  public double peakOutput;

  public Gains(double P, double I, double D, double F, double integralZone, double peakOutput) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.F = F;
    this.integralZone = integralZone;
    this.peakOutput = peakOutput;
  }
}

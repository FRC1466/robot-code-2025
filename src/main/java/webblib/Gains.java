// Copyright (c) 2025 FRC 1466
// https://github.com/FRC1466
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

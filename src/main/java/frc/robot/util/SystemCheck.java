// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class SystemCheck {
  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private static final double TEST_DURATION = 1.0; // Duration for each motor test
  private static final double MOTOR_TEST_SPEED = 0.2; // 20% speed for testing
  private static final String CONTINUE_KEY = "SystemCheck/Continue";
  private static final double WAIT_TIMEOUT = 30.0; // Timeout for operator response

  public SystemCheck(CommandSwerveDrivetrain drivetrain, Elevator elevator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    SmartDashboard.putBoolean(CONTINUE_KEY, false);
  }

  private Command waitForOperatorConfirm(String message) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              SmartDashboard.putBoolean(CONTINUE_KEY, false);
              Logger.recordOutput(
                  "SystemCheck/Status", message + " - Waiting for operator confirmation");
            }),
        Commands.waitUntil(() -> SmartDashboard.getBoolean(CONTINUE_KEY, false))
            .withTimeout(WAIT_TIMEOUT));
  }

  // Run sequential system checks
  public Command runSystemCheck() {
    return Commands.sequence(
        // Initial status
        Commands.runOnce(() -> Logger.recordOutput("SystemCheck/Status", "Starting System Check")),

        // 1. Test drive motors individually
        waitForOperatorConfirm("Ready to test drive motors"),
        testDriveMotors(),

        // 2. Test turn motors individually
        waitForOperatorConfirm("Ready to test turn motors"),
        testTurnMotors(),

        // 3. Test elevator
        waitForOperatorConfirm("Ready to test elevator"),
        testElevatorSetpoints(),

        // 4. Test arm
        waitForOperatorConfirm("Ready to test arm"),
        testArmSetpoints(),

        // 5. Test intake and sensor
        waitForOperatorConfirm("Ready to test intake and sensors"),
        testIntakeAndSensor(),

        // Final status
        Commands.runOnce(
            () -> {
              Logger.recordOutput("SystemCheck/Status", "System Check Complete");
              SmartDashboard.putBoolean(CONTINUE_KEY, false);
            }));
  }

  private Command testDriveMotors() {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("SystemCheck/Status", "Testing Drive Motors")),
        // Test each drive motor with confirmation
        waitForOperatorConfirm("Testing Front Left Drive Motor"),
        driveMotorTest(0),
        waitForOperatorConfirm("Testing Front Right Drive Motor"),
        driveMotorTest(1),
        waitForOperatorConfirm("Testing Back Left Drive Motor"),
        driveMotorTest(2),
        waitForOperatorConfirm("Testing Back Right Drive Motor"),
        driveMotorTest(3));
  }

  private Command driveMotorTest(int moduleNumber) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var module = drivetrain.getModule(moduleNumber);
              module.getDriveMotor().set(MOTOR_TEST_SPEED);
            }),
        new WaitCommand(TEST_DURATION),
        Commands.runOnce(
            () -> {
              var module = drivetrain.getModule(moduleNumber);
              module.getDriveMotor().set(0);
            }));
  }

  private Command testTurnMotors() {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("SystemCheck/Status", "Testing Turn Motors")),
        // Test each turn motor with confirmation
        waitForOperatorConfirm("Testing Front Left Turn Motor"),
        turnMotorTest(0),
        waitForOperatorConfirm("Testing Front Right Turn Motor"),
        turnMotorTest(1),
        waitForOperatorConfirm("Testing Back Left Turn Motor"),
        turnMotorTest(2),
        waitForOperatorConfirm("Testing Back Right Turn Motor"),
        turnMotorTest(3));
  }

  private Command turnMotorTest(int moduleNumber) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var module = drivetrain.getModule(moduleNumber);
              module.getSteerMotor().set(MOTOR_TEST_SPEED);
            }),
        new WaitCommand(TEST_DURATION),
        Commands.runOnce(
            () -> {
              var module = drivetrain.getModule(moduleNumber);
              module.getSteerMotor().set(0);
            }));
  }

  private Command testElevatorSetpoints() {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("SystemCheck/Status", "Testing Elevator")),
        waitForOperatorConfirm("Moving elevator to L2"),
        elevator.toL2(),
        new WaitCommand(2.0),
        waitForOperatorConfirm("Moving elevator to bottom"),
        elevator.toBottom());
  }

  private Command testArmSetpoints() {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("SystemCheck/Status", "Testing Arm")),
        waitForOperatorConfirm("Moving arm to store position"),
        Commands.runOnce(() -> RobotContainer.rotaryPart.store().withTimeout(0.01)),
        new WaitCommand(2.0),
        waitForOperatorConfirm("Moving arm to coral score position"),
        Commands.runOnce(() -> RobotContainer.rotaryPart.coralScore().withTimeout(0.01)),
        new WaitCommand(2.0),
        waitForOperatorConfirm("Moving arm to algae grab position"),
        Commands.runOnce(() -> RobotContainer.rotaryPart.algaeGrab().withTimeout(0.01)));
  }

  private Command testIntakeAndSensor() {
    return Commands.sequence(
        Commands.runOnce(
            () -> Logger.recordOutput("SystemCheck/Status", "Testing Intake & Sensor")),
        waitForOperatorConfirm("Starting intake test - place object in front of sensor"),
        RobotContainer.intake.intake(),
        Commands.waitUntil(() -> !RobotContainer.intake.getIntakeDistanceBool()),
        RobotContainer.intake.stop(),
        waitForOperatorConfirm("Intake test complete"));
  }
}

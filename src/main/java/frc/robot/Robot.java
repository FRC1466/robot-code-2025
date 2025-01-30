// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix ;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
                .getStructTopic("Test", Pose2d.struct)
                .publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    var visionEst = RobotContainer.photonCamera.getEstimatedGlobalPose(); 
   

      
     
        visionEst.ifPresent(
                 est -> {
                                 
                    var estStdDevs = RobotContainer.photonCamera.getEstimationStdDevs();
                    SmartDashboard.putNumber( "Timestamp",est.timestampSeconds);
                    SmartDashboard.putNumber("Vision Pose X", visionEst.get().estimatedPose.toPose2d().getX());
                    SmartDashboard.putNumber("Vision Pose Y", visionEst.get().estimatedPose.toPose2d().getY());
                    //RobotContainer.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    RobotContainer.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                 });
        //SmartDashboard.putBoolean("Camera?", photonCamera.getCamera().isConnected());
        SmartDashboard.putBoolean( "Vision Est",visionEst.isPresent());
        SmartDashboard.putNumber("Robot Pose X", RobotContainer.drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("Robot Pose Y", RobotContainer.drivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("Robot Pose Theta", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
  }
 
  public void periodic(){
       
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
      /*SmartDashboard.putData
    ("Robot Pose", Telemetry.telemeterize.getPose());*/
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

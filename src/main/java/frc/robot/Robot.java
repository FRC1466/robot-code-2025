// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix ;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends LoggedRobot {

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d RobotPose;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
                .getStructTopic("Test", Pose2d.struct)
                .publish();

  public Robot() {
    
    m_robotContainer = new RobotContainer();
    AutoLogOutputManager.addObject(this); // Add this object for logging


    Logger.recordMetadata("ProjectName", "Robot-Code-2025"); // Set a metadata value

  if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    
  } else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
  }

Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  @Override
  public void robotInit() {
      m_robotContainer.uppy.setSelectedSensorPosition(0);
      
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
        //SmartDashboard.pu Boolean("Camera?", photonCamera.getCamera().isConnected());
        SmartDashboard.putBoolean( "Vision Est",visionEst.isPresent());
        RobotPose = RobotContainer.drivetrain.getState().Pose;
        SmartDashboard.putNumber("Robot Pose X", RobotContainer.drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("Robot Pose Y", RobotContainer.drivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("Robot Pose Theta", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        
      SwerveDriveState driveState = RobotContainer.drivetrain.getState();
      Logger.recordOutput("SwerveModuleStates", driveState.ModuleStates);
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

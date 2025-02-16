// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTester;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.RotatyPart;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(.1).withRotationalDeadband(.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final Intake intake = new Intake();

    public final RotatyPart rotatyPart = new RotatyPart();
    final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandJoystick joystick = new CommandJoystick(0);

    public static CommandSwerveDrivetrain drivetrain;
    public final static Vision photonCamera = new Vision();
    public final static Elevator uppy = new Elevator();
    public static boolean sliderEnabled = false;


    public RobotContainer() {
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getRobot()) {
                          case COMPBOT -> {
                            drivetrain = TunerConstants.createDrivetrain();
                          }
                          case DEVBOT -> {
                            drivetrain = TunerConstantsTester.createDrivetrain();
                          }
                            case SIMBOT -> throw new UnsupportedOperationException("Unimplemented case: " + Constants.getRobot());
                            default -> throw new IllegalArgumentException("Unexpected value: " + Constants.getRobot());
            }
        }
        configureBindings();
        initializeChooser();
    }
  
    public void addChooser(String name, String autoName){
        autoChooser.addOption(name, new PathPlannerAuto(autoName));
    }


    public void initializeChooser(){
    addChooser("Recenter bot", "Recenter");
    addChooser("Best 3 Piece", "3 Piece Auto Better");
    addChooser("2 Piece Inverted", "Invert 2 Piece Auto");
    autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));
    autoChooser.addOption("2 Piece", new PathPlannerAuto("2 Piece Auto"));
    autoChooser.addOption("Taxi PID Testing", new PathPlannerAuto("PID Testing"));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Piece Auto"));
   
    
    
  NamedCommands.registerCommand("Raise arm to Station", uppy.toStation());
  NamedCommands.registerCommand( "raise arm to 1", uppy.toL1());
  NamedCommands.registerCommand( "raise arm to 2", uppy.toL2());
  NamedCommands.registerCommand( "raise arm to 3", uppy.toL3());
  NamedCommands.registerCommand( "raise arm to 4", uppy.toL4());
  NamedCommands.registerCommand("intake coral", intake.intake());
  NamedCommands.registerCommand( "reverse intake coral", intake.reverseIntake());
  NamedCommands.registerCommand( "rotary part", rotatyPart.coralScore());
  
  
  
  SmartDashboard.putData("CHOOSE", autoChooser);
    
}

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Math.pow((MathUtil.applyDeadband(-joystick.getY(), .05)),3) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Math.pow((MathUtil.applyDeadband(-joystick.getX(), .05)),3) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Math.pow((MathUtil.applyDeadband(-joystick.getZ(), .05)),3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // reset the field-centric heading on left bumper press
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                 
        //Intake Coral
        joystick.button(1).onTrue(uppy.toBottom().alongWith(rotatyPart.store()).alongWith(intake.intake())).onFalse(intake.stop());
        //L2
        joystick.button(3).onTrue(uppy.toL2().alongWith(rotatyPart.coralScore())).onFalse(intake.intake().alongWith(Commands.waitSeconds(2)).andThen(uppy.toBottom()).andThen(intake.stop()));
        //L3
        joystick.button(2).onTrue(uppy.toL3().alongWith(rotatyPart.coralScore())).onFalse(intake.intake().alongWith(Commands.waitSeconds(2)).andThen(uppy.toBottom()).andThen(intake.stop()));
        //L4
        joystick.button(4).onTrue(rotatyPart.coralScore().alongWith(intake.hold()).alongWith(Commands.waitSeconds(.5)).andThen(uppy.toL4()).alongWith(Commands.waitSeconds(4)).andThen(rotatyPart.l4coralScore())).onFalse(intake.intake().alongWith(Commands.waitSeconds(2)).andThen(rotatyPart.coralScore()).andThen(uppy.toBottom()).andThen(intake.stop()));
        //joystick.button(2).onTrue(intake.intake()).onFalse(intake.stop());
        //joystick.button(3).onTrue(rotatyPart.l4coralScore()).onFalse(rotatyPart.store());
        //joystick.button(4).onTrue(rotatyPart.algaeGrab()).onFalse(rotatyPart.store());
        joystick.button(5).onTrue(rotatyPart.coralScore()).onFalse(rotatyPart.store());
        joystick.button(6).onTrue(intake.reverseIntake()).onFalse(intake.stop());
        joystick.button(7).onTrue(intake.hold()).onFalse(intake.stop());
        joystick.button(9).onTrue(uppy.toL2()).onFalse(uppy.toBottom());
        joystick.button(11).onTrue(uppy.toL4()).onFalse(uppy.toBottom());
        joystick.button(12).onTrue(switchState(true)).onFalse(switchState(false));

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command switchState(boolean bool){
        return run(() -> changeState(bool));
    }

    public void changeState(boolean bool){
        sliderEnabled = bool;
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}

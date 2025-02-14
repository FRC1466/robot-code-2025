package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;

public class Elevator extends SubsystemBase {



    private final TalonFX masterMotor, leftSlaveFX;
    private static Elevator instance;
    //Will use when we install hall sensor
    //DigitalInput hallBottom = new DigitalInput(0);


    //figure out what this is
    private static final double TICKS_PER_INCH = 1;
    //Also figure this out
    private static final double HOME_POSITION_INCHES = 4;

    public Elevator() {
        masterMotor = new TalonFX(Constants.Elevator.masterID);
        
        leftSlaveFX = new TalonFX(Constants.Elevator.slaveID);

       masterMotor.setVoltage(0);
       leftSlaveFX.setVoltage(0);
       setNeutralMode(NeutralModeValue.Brake);
    }


    private void setNeutralMode(NeutralModeValue neutralMode){
        masterMotor.setNeutralMode(neutralMode);
        leftSlaveFX.setNeutralMode(neutralMode);
    }

    public void setSelectedSensorPosition(double position){
        masterMotor.setPosition(position);
        leftSlaveFX.setPosition(position);
    }

    public double getElevatorHeight(){
        double height = masterMotor.getPosition().getValueAsDouble();
        return height;
    }

    public void setElevatorPose(double pose){
        pose = (pose-1)*-.5;
        masterMotor.setPosition(pose);
    }

    public Command runElevator(double pose){
        return runOnce(() -> setElevatorPose(pose));
    }

    public void setMotorVoltage(double volts){
        masterMotor.setVoltage(volts);
        leftSlaveFX.setVoltage(-volts);
    }

    public Command setElevatorVoltage(double volts){
        return runOnce(() -> setMotorVoltage(volts));
    }

    @Override
    public void periodic(){


         //if(!hallBottom.get()){
         //   setSelectedSensorPosition(0);
        //}

        Logger.recordOutput("Elevator Position", getElevatorHeight());

    }
}

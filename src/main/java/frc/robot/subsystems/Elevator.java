package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final TalonFX masterMotor, leftSlaveFX;
    private static Elevator instance;
    //Will use when we install hall sensor
    //private final DigitalInput hallBottom;

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

    public Command setElevatorVoltage(double volts){
        return runOnce(() -> setMotorVoltage(volts));
    }

    public void setMotorVoltage(double volts){
        masterMotor.setVoltage(volts);
        leftSlaveFX.setVoltage(-volts);
    }

    @Override
    public void periodic(){

       /*  if(!hallBottom.get()){
            setSelectedSensorPosition(0);
        }*/
        SmartDashboard.putNumber("Elevator Position", getElevatorHeight());

    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import  frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private TalonFX turretMotor;
    
    public Turret(){
        turretMotor = new TalonFX(Constants.TURRET_MOTOR); 
        turretMotor.configPeakOutputForward(1);
        turretMotor.configPeakOutputReverse(-1);
        turretMotor.configFactoryDefault();
    }

    @Override
    public void periodic(){
        
    }

    public void setTurretSpeed(double speed){
        turretMotor.set(ControlMode.PercentOutput, speed);
    }

    public double readEncoder(){
        return turretMotor.getSelectedSensorPosition();
    }
    
}

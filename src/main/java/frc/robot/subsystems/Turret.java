package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private CANCoder encoder;

    private TalonFX turretMotor;
    
    public Turret(){
        encoder = new CANCoder(0); // We will set the CONSTANTS Value when we have it
        

        turretMotor = new TalonFX(0); //Change ID when known
        turretMotor.configPeakOutputForward(1);
        turretMotor.configPeakOutputReverse(-1);
    }

    @Override
    public void periodic(){

    }

    public void setTurretSpeed(double speed){
        turretMotor.set(ControlMode.PercentOutput, speed);
    }

    public double readEncoder(){
        return encoder.getPosition(); 
    }
    
}

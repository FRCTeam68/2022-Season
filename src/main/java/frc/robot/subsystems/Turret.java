package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import  frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private TalonFX turretMotor;
    
    public Turret(){
        turretMotor = new TalonFX(Constants.TURRET_MOTOR); 
        turretMotor.configPeakOutputForward(1);
        turretMotor.configPeakOutputReverse(-1);
        turretMotor.configFactoryDefault();

        //soft limits
        turretMotor.configForwardSoftLimitThreshold(14336); //180-deg limit with ring gear 140, spur 10, gearbox 45, encoder 1024
        turretMotor.configReverseSoftLimitThreshold(-14336);
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);

        /* newer config API */
			TalonFXConfiguration configs = new TalonFXConfiguration();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			/* config all the settings */
			turretMotor.configAllSettings(configs);
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
/*
    public void setTurretPosition(double position) {
        turretMotor.set(ControlMode.Position, position );
    }
*/    
    public void ResetEncoders(){
        turretMotor.setSelectedSensorPosition(0,0,0);
      }
    
}

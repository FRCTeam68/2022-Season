package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

  private TalonFX shooter;

  private double gP = 0;
  private double gI = 0;
  private double gD = 0;
  private double gF = 0; //Feed Forward
  private double gRef = 0; //Setpoint
  private double shootSpeed = 1;

  public Shooter() {
    //TalonFX Initialization
    shooter.configFactoryDefault();
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    shooter.set(ControlMode.Velocity,0);

    shooter.config_kP(0, gP);
    shooter.config_kI(0, gI);
    shooter.config_kD(0, gD);
    shooter.config_kF(0, gF);

    shooter.setNeutralMode(NeutralMode.Coast);

    shooter.setSensorPhase(true);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    //method for editing PID values on SmartDashboard            
    editable();
                
    //run motor after setting setpoint values
    speedByEncoder();       

    //getting encoder values 
    double encValue = shooter.getSensorCollection().getIntegratedSensorVelocity();
    double normalizedRPM = (encValue/2048)*600;

    
    //display encoder values on SmartDashboard
    SmartDashboard.putNumber("encoder value(RPM)", normalizedRPM);
    SmartDashboard.putNumber("Raw encoder value", encValue);
  }

  public void editable() {
    
    // get edited PID values from smart dashboard
    double P = SmartDashboard.getNumber("P Input", gP);
    double I = SmartDashboard.getNumber("I Input", gI);
    double D = SmartDashboard.getNumber("D Input", gD);
    double F = SmartDashboard.getNumber("F Input", gF);
    double Ref = SmartDashboard.getNumber("Setpoint Input", gRef);

    if(gP != P) {

      gP = P;
     shooter.config_kP(0, gP);

    }

    if(gI != I){

      gI = I;
      shooter.config_kI(0, gI);

  

    }

    if(gD != D){

      gD = D;
      shooter.config_kD(0, gD);

    }

    if(gF != F){

      gF = F;
      shooter.config_kF(0,gF);

    }

    if(gRef != Ref){

      gRef = Ref; 

    }

    SmartDashboard.putNumber("P Input", gP);
    SmartDashboard.putNumber("I Input", gI);
    SmartDashboard.putNumber("D Input", gD);
    SmartDashboard.putNumber("F Input", gF);
    SmartDashboard.putNumber("Setpoint Input", gRef);
  }

  public void speedByEncoder() {

    // turning set RPM into encoder input
    double encFeedValue = (gRef/600)*2048/0.5; //in place of the 1, put in the gear ratio
    shooter.set(ControlMode.Velocity, encFeedValue);
    
    SmartDashboard.putNumber("encoderFeedValue", encFeedValue);
  }
  
  public void zeroEncoders(){
    shooter.set(ControlMode.Position, 0); //Maybe works?
    shooter.setSelectedSensorPosition(0); // Test at some point
  }

  public void setSpeed(double speed){
    shooter.set(ControlMode.PercentOutput, speed);
  }
}
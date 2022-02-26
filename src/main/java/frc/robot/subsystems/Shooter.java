package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

  private TalonFX shooterLeft;
  private TalonFX shooterRight;
  private double gP = 0;
  private double gI = 0;
  private double gD = 0;
  private double gF = 0; //Feed Forward
  private double gRef = 0; //Setpoint
  private double shootSpeed = 1;

  public Shooter() {
    //TalonFX Initialization
    shooterLeft = new TalonFX(9); 
    shooterRight = new TalonFX(10); 
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();
    shooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    shooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    shooterLeft.set(ControlMode.Velocity,0);
    shooterLeft.config_kP(0, gP);
    shooterLeft.config_kI(0, gI);
    shooterLeft.config_kD(0, gD);
    shooterLeft.config_kF(0, gF);

    shooterRight.set(ControlMode.Velocity,0);
    shooterRight.config_kP(0, gP);
    shooterRight.config_kI(0, gI);
    shooterRight.config_kD(0, gD);
    shooterRight.config_kF(0, gF);

    shooterLeft.setNeutralMode(NeutralMode.Coast);

    shooterLeft.setSensorPhase(true);

    shooterRight.setNeutralMode(NeutralMode.Coast);

    shooterRight.setSensorPhase(true);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    //method for editing PID values on SmartDashboard            
    editable();
                
    //run motor after setting setpoint values
    speedByEncoder();       

    //getting encoder values 
    double encValueLeft = shooterLeft.getSensorCollection().getIntegratedSensorVelocity();
    double normalizedRPMLeft = (encValueLeft/2048)*600;

    double encValueRight = shooterRight.getSensorCollection().getIntegratedSensorVelocity();
    double normalizedRPMRight = (encValueRight/2048)*600;

    //display encoder values on SmartDashboard
    SmartDashboard.putNumber("encoder value(RPM)", normalizedRPMLeft);
    SmartDashboard.putNumber("Raw encoder value", encValueLeft);
    SmartDashboard.putNumber("encoder value(RPM)", normalizedRPMRight);
    SmartDashboard.putNumber("Raw encoder value", encValueRight);
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
     shooterLeft.config_kP(0, gP);
     shooterRight.config_kP(0, gP);
    }

    if(gI != I){

      gI = I;
      shooterLeft.config_kI(0, gI);
      shooterRight.config_kI(0, gI);

  

    }

    if(gD != D){

      gD = D;
      shooterLeft.config_kD(0, gD);
      shooterRight.config_kD(0, gD);
    }

    if(gF != F){

      gF = F;
      shooterLeft.config_kF(0, gD);
      shooterRight.config_kF(0, gD);
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
    shooterLeft.set(ControlMode.Velocity, -encFeedValue);
    shooterRight.set(ControlMode.Velocity, encFeedValue);
    SmartDashboard.putNumber("encoderFeedValue", encFeedValue);
  }
  
  public void zeroEncoders(){
    shooterLeft.set(ControlMode.Position, 0); //Maybe works?
    shooterLeft.setSelectedSensorPosition(0); // Test at some point
    shooterRight.set(ControlMode.Position, 0); //Maybe works?
    shooterRight.setSelectedSensorPosition(0); // Test at some point
  }

  public void setSpeed(double speed){
    shooterLeft.set(ControlMode.PercentOutput, -speed);
    shooterRight.set(ControlMode.PercentOutput, speed);
    
  }
}
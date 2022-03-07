package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import  frc.robot.Constants;

public class Shooter extends SubsystemBase{

  private TalonFX shooterLeft;
  private TalonFX shooterRight;
  private double gP = 0;
  private double gI = 0;
  private double gD = 0;
  private double gF = 0; //Feed Forward
  private double gRef = 0; //Setpoint
  private double shootSpeed = 1;
  private SimpleMotorFeedforward feedForward;
  private PIDController shooterPID;

  public Shooter() {
    //TalonFX Initialization
    shooterLeft = new TalonFX(Constants.LEFT_SHOOTER_MOTOR); 
    shooterRight = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR); 
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
    //feedForward = new SimpleMotorFeedforward(-5.0424, 0.0002596, 0.0030056);
    feedForward = new SimpleMotorFeedforward(0.133, 0.0002596, 0.0030056);
    shooterPID = new PIDController(0.05, 0, 0); //0.14258
  }

  @Override
  public void periodic() {

  }
/*
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
*/
  
  
  public void zeroEncoders(){
    shooterLeft.set(ControlMode.Position, 0); //Maybe works?
    shooterLeft.setSelectedSensorPosition(0); // Test at some point
    shooterRight.set(ControlMode.Position, 0); //Maybe works?
    shooterRight.setSelectedSensorPosition(0); // Test at some point
  }

  public void setSpeed(double speed){
    shooterLeft.set(ControlMode.Velocity, -speed);
    shooterRight.set(ControlMode.Velocity, speed);
    
  }

  public void shooterFeedForward(double velocity){
    shooterLeft.set(ControlMode.PercentOutput, -feedForward.calculate(velocity));
    //+ shooterPID.calculate(shooterLeft.getSelectedSensorVelocity(), velocity));
    shooterRight.set(ControlMode.PercentOutput, feedForward.calculate(velocity));
    //+ shooterPID.calculate(shooterRight.getSelectedSensorVelocity(), velocity));
  }
}
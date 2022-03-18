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
import frc.robot.Robot;
import frc.robot.commands.Shoot;

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

  private double numerator;
 private double denominator;
 private double frac;
 private double circball;
 private double circwheel;
 private double Vi;
 private final double GRAVITY = -32;
 private double xDisplacement = 0;
 private final double LIMELIGHT_HEIGHT_OFF_GROUND = 41.375; //measure, in inches
 private double angleToGoal;
 private double rpsball;
 private double rps_ratio;
 private double rpsflywheel;
 private static double rpm = 0; 
 private double kP, kI, kD, kF;

  public Shooter() {

    kP = 0.0465;
    kI = 0.0005;
    kD = 0.0;
    kF = 0.060;
    //TalonFX Initialization
    shooterLeft = new TalonFX(Constants.LEFT_SHOOTER_MOTOR); 
    shooterRight = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR); 
    //shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();
    //shooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    shooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    shooterLeft.follow(shooterRight);
    shooterLeft.setInverted(true);
    

    shooterRight.set(ControlMode.Velocity,0);
    shooterRight.config_kP(0, kP);
    shooterRight.config_kI(0, kI);
    shooterRight.config_kD(0, kD);
    shooterRight.config_kF(0, kF);

    shooterLeft.setNeutralMode(NeutralMode.Coast);

    shooterLeft.setSensorPhase(true);

    shooterRight.setNeutralMode(NeutralMode.Coast);

    shooterRight.setSensorPhase(true);
    //feedForward = new SimpleMotorFeedforward(-5.0424, 0.0002596, 0.0030056);
    feedForward = new SimpleMotorFeedforward(0.133, 0.0002596, 0.0030056);
    shooterPID = new PIDController(0.05, 0, 0); //0.14258
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Shoot());
}
  @Override
  public void periodic() {

  }

  
  public void zeroEncoders(){
    shooterLeft.set(ControlMode.Position, 0); //Maybe works?
    shooterLeft.setSelectedSensorPosition(0); // Test at some point
    shooterRight.set(ControlMode.Position, 0); //Maybe works?
    shooterRight.setSelectedSensorPosition(0); // Test at some point
  }

  public void setSpeed(double speed){
    
    shooterRight.set(ControlMode.Velocity, speed);
    
  }

  public void shooterFeedForward(double velocity){

    shooterRight.set(ControlMode.PercentOutput, feedForward.calculate(velocity));
    //+ shooterPID.calculate(shooterRight.getSelectedSensorVelocity(), velocity));
  }

  public double m_calculateRPM(){
    
    
    xDisplacement = Robot.vision.calcDistance();
  
    numerator = GRAVITY * xDisplacement * xDisplacement;
    denominator = 2 * (LIMELIGHT_HEIGHT_OFF_GROUND - (xDisplacement * Math.tan(Constants.THETA))) * Math.pow(Math.cos(Constants.THETA), 2);
    frac = numerator / denominator;
    Vi = Math.sqrt(frac);

    circball = (2 * Math.PI * Constants.COMPRESSED_RADIUS) / 12.0; //ft
    circwheel = (2 * Math.PI * Constants.FLYWHEEL_RADIUS) /12.0; //ft

    rpsball = Vi / circball; //rotations per second

    rps_ratio = (circball / circwheel); //ratio of ball rpm to wheel rpm

    rpsflywheel = rpsball * rps_ratio / Constants.SLIPPERINESS; //rotations per second

    rpm = 60 * rpsflywheel;
    
    return rpm;
  }
  public void setRPM(double wheelRPM){
    //Sensor Velocity in ticks per 100ms / Sensor Ticks per Rev * 600 (ms to min) * 2 gear ratio to shooter
    //Motor Velocity in RPM / 600 (ms to min) * Sensor ticks per rev / pulley Ratio 36 to 18
    double motorVelocity = (wheelRPM / 600 * 2048) / 2;
    setSpeed(motorVelocity);
  }
  public double getWheelRPM(){
    return rpm;
  }
  public boolean goodToShoot(double target){
    boolean isTrue = false;
    if((target-100)<getWheelRPM()&&getWheelRPM()>(target+100)){
      isTrue = true;
    }
    else{
      isTrue = false;
    }
    return isTrue;
  }
}
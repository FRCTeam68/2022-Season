
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;

public class Endgame extends SubsystemBase {
  /** Creates a new Endgame. */

  private TalonFX rightArm, leftArm;

  public double currentR = 0, currentL = 0;

  private double kP, kI, kD, kF;

  public Endgame() {

    kP = 0.0465;
    kI = 0.0005;
    kD = 0.0;
    kF = 0.060;
    //TalonFX Initialization
    rightArm = new TalonFX(Constants.RIGHT_ARM_MOTOR);
    leftArm = new TalonFX(Constants.LEFT_ARM_MOTOR);
    //shooterLeft.configFactoryDefault();
    rightArm.configFactoryDefault();
    leftArm.configFactoryDefault();
    rightArm.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
    //shooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    // rightArm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    

    // rightArm.set(ControlMode.Velocity,0);
    // rightArm.config_kP(0, kP);
    // rightArm.config_kI(0, kI);
    // rightArm.config_kD(0, kD);
    // rightArm.config_kF(0, kF);

    // leftArm.setNeutralMode(NeutralMode.Brake);

    // leftArm.setSensorPhase(true);

    // rightArm.setNeutralMode(NeutralMode.Brake);

    // rightArm.setSensorPhase(true);

    rightArm.selectProfileSlot(Constants.RARM_PID_SLOT, 0);
		rightArm.config_kF(Constants.RARM_PID_SLOT, Constants.RARM_PID_F, 0);
		rightArm.config_kP(Constants.RARM_PID_SLOT, Constants.RARM_PID_P, 0);
		rightArm.config_kI(Constants.RARM_PID_SLOT, Constants.RARM_PID_I, 0);
		rightArm.config_kD(Constants.RARM_PID_SLOT, Constants.RARM_PID_D, 0);

    leftArm.selectProfileSlot(Constants.RARM_PID_SLOT, 0);
		leftArm.config_kF(Constants.RARM_PID_SLOT, Constants.RARM_PID_F, 0);
		leftArm.config_kP(Constants.RARM_PID_SLOT, Constants.RARM_PID_P, 0);
		leftArm.config_kI(Constants.RARM_PID_SLOT, Constants.RARM_PID_I, 0);
		leftArm.config_kD(Constants.RARM_PID_SLOT, Constants.RARM_PID_D, 0);
  }

  @Override
  public void periodic() {
    currentR = rightArm.getStatorCurrent();
    currentL = leftArm.getStatorCurrent();
  }

  public double getLeftEncoder(){
    return leftArm.getSelectedSensorPosition();
  }

  public double getRightEncoder(){
    return rightArm.getSelectedSensorPosition();
  }
  public void ResetEncoders(){
    leftArm.setSelectedSensorPosition(0,0,0);
    rightArm.setSelectedSensorPosition(0,0,0);
  }

  public void setLeftPos(double pos){
    
    leftArm.set(ControlMode.Position, pos);
    
  }
  public void setRightPos(double pos){
    
    rightArm.set(ControlMode.Position, pos);
    
  }
}
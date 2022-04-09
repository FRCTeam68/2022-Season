// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;

public class Endgame extends SubsystemBase {
  /** Creates a new Endgame. */

  private TalonFX rightArm, leftArm;

  public Orchestra sounds;

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
    //feedForward = new SimpleMotorFeedforward(-5.0424, 0.0002596, 0.0030056);

    //sounds.addInstrument(leftArm);
    //sounds.loadMusic("deply/tester.chrp");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public void setLeftSpeed(double speed){
    leftArm.set(ControlMode.PercentOutput, speed);
  }
  public void setRightSpeed(double speed){
    rightArm.set(ControlMode.PercentOutput, speed);
  }

  public double getAngleToGround(){
    return RobotContainer.m_drivetrainSubsystem.m_navx.getRawGyroX(); // I dont know the actual angle we need to measure so TEST TEST TEST TEST TEST!
  }

}

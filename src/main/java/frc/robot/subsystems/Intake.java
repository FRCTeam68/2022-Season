// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.DigitalGlitchFilterJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import  frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX indexMotorHigh;
  private TalonFX indexMotorLow;

  private DigitalInput intakeSensor;
  private DigitalInput indexSensor;

  public static Intake intake;

  public static Intake getIntake(){
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  /** Creates a new Intake. */
  public Intake() {


    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR); // Intake Motor (Change value when known)
    intakeMotor.configPeakOutputForward(1);
    intakeMotor.configPeakOutputReverse(-1);

    indexMotorHigh = new TalonFX(Constants.INDEX_MOTOR_HIGH); // Change value when known
    indexMotorHigh.configPeakOutputForward(1);
    indexMotorHigh.configPeakOutputReverse(-1);

    indexMotorLow = new TalonFX(Constants.INDEX_MOTOR_LOW); // Change value when known
    indexMotorLow.configPeakOutputForward(1);
    indexMotorLow.configPeakOutputReverse(-1);

    intakeSensor = new DigitalInput(Constants.INTAKE_SENSOR);
    indexSensor = new DigitalInput(Constants.INDEX_SENSOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //CommandScheduler.getInstance().setDefaultCommand(Robot.intake, new IntakeCommand());
  }

  public boolean getIntakeBeamBreak(){
    return intakeSensor.get();
  } 

  public boolean getIndexBeamBreak(){
    return indexSensor.get();
  } 
  
  public void setIntakeSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setIndexHighSpeed(double speed){
    indexMotorHigh.set(ControlMode.PercentOutput, speed);
  }
  public void setIndexLowSpeed(double speed){
    indexMotorLow.set(ControlMode.PercentOutput, speed);
  }

  // Have a combined function because maybe you want to change both at the same time??

  public void setSpeedOfIndexAndIntake(double indexHighSpeed, double indexLowSpeed, double intakeSpeed){
    intakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    indexMotorLow.set(ControlMode.PercentOutput, -indexLowSpeed);
    indexMotorHigh.set(ControlMode.PercentOutput, indexHighSpeed);
  }

}

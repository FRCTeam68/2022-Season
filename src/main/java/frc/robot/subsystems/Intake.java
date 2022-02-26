// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommand;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX indexMotor;

  //private Orchestra soundTest;

  // Colour sensors with their colours
  private ColorSensorV3 topSensor;
  private ColorSensorV3 lowSensor; 

  private I2C.Port topSensorPort;
  private I2C.Port lowSensorPort;

  public static Intake intake;

  public static Intake getIntake(){
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  /** Creates a new Intake. */
  public Intake() {

 //   soundTest = new Orchestra();
    intakeMotor = new TalonFX(12); // Intake Motor (Change value when known)
    intakeMotor.configPeakOutputForward(1);
    intakeMotor.configPeakOutputReverse(-1);
/*
    soundTest.addInstrument(intakeMotor);
    soundTest.loadMusic("deploy\\tester.chrp"); // If you get an error here its because I made a mistake. 
    // I probably didn't put the music in the correct path
*/
    indexMotor = new TalonFX(14); // Change value when known
    indexMotor.configPeakOutputForward(1);
    indexMotor.configPeakOutputReverse(-1);

    //Init Color Sensors
    topSensorPort = I2C.Port.kOnboard;
    lowSensorPort = I2C.Port.kOnboard;

    topSensor = new ColorSensorV3(topSensorPort);
    lowSensor = new ColorSensorV3(lowSensorPort);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //CommandScheduler.getInstance().setDefaultCommand(Robot.intake, new IntakeCommand());
  }

  public boolean doesTopSense(){
    if (getTopIR() < 0.1) //I don't really know the value here. Will need some testing I guess
      return true;
    
    return false;
  }

  // Returns the infered approx. for the top sensor. Higher it is, the closer the object
  public int getTopIR(){
    return topSensor.getIR();
  }

  // Returns the infered approx. for the low sensor. Higher it is, the closer the object
  public int getLowIR(){
    return lowSensor.getIR();
  }

  public Color getTopColor(){
    return topSensor.getColor();
  }

  public Color getLowColor(){
    return lowSensor.getColor();
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setIndexSpeed(double speed){
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  // Have a combined function because maybe you want to change both at the same time??

  public void setSpeedOfIndexAndIntake(double indexSpeed, double intakeSpeed){
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    indexMotor.set(ControlMode.PercentOutput, indexSpeed);
  }
/*
  public void toggleMusic(){
    if (!soundTest.isPlaying())
      soundTest.play();
    else
      soundTest.stop();
  }
*/
}

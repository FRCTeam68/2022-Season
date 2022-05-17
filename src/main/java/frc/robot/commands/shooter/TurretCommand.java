// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class TurretCommand extends CommandBase {

  double rightX;
  public TurretCommand() {
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      //rightX = Robot.m_robotContainer.getRightManipulatorJoystickValue();
      Robot.turret.setTurretSpeed(Robot.vision.steeringAdjust());
      //Robot.turret.setTurretSpeed(rightX);
      
  }
  

  @Override
  public void end(boolean interrupted) {
    Robot.turret.setTurretSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
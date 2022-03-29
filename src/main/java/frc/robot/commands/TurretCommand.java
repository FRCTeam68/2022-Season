// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  double rightX;
  public TurretCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //rightX = Robot.m_robotContainer.getRightManipulatorJoystickValue();
      Robot.turret.setTurretSpeed(Robot.vision.steeringAdjust());
      //Robot.turret.setTurretSpeed(rightX);
      
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.turret.setTurretSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
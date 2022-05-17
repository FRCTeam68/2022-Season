// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {

  public Shoot() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
     //2048 ticks per revolution; 600 goes from minutes to 100ms; 2000 rpm is goal  wall 4400 initiationLine 3700 low 2000
    Robot.shooter.setRPM(Robot.shooter.m_calculateRPM());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setRPM(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
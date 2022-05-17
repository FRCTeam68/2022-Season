// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.other;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CameraModeOff extends CommandBase {

  boolean isFinish = false;
  public CameraModeOff() {

  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    Robot.vision.setCameraMode(1,1);
    //isFinish = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinish;
  }
}

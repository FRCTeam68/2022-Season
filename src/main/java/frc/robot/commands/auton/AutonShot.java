// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutonShot extends CommandBase {
  boolean isFinished = false;
  public AutonShot() {
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.shooter.setRPM(Constants.AUTON_SHOT);
    isFinished = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

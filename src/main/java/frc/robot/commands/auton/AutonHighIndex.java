// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutonHighIndex extends CommandBase {

  boolean isfinished = false;
  public AutonHighIndex() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.intake.setSpeedOfIndexAndIntake(.4, 0, 0);
    isfinished=true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isfinished;
  }
}

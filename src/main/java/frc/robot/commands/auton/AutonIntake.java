// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutonIntake extends CommandBase {

  public AutonIntake() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.intake.setSpeedOfIndexAndIntake(.4, .4, .4);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

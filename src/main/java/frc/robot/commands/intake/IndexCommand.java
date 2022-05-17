// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IndexCommand extends CommandBase {
  public IndexCommand() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.intake.setSpeedOfIndexAndIntake(.5, .2, 0);
    /*
    if(Robot.shooter.goodToShoot(Constants.shooterTargetRPM())){
      Robot.intake.setSpeedOfIndexAndIntake(.5, .2, 0);
    }
    else{
      Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
    }
    */
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

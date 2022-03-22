// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IndexCommand extends CommandBase {
  /** Creates a new IndexCommand. */
  public IndexCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

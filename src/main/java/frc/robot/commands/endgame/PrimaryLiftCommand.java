// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endgame;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrimaryLiftCommand extends CommandBase {
  /*
  WARNING!!!!!!!!!
  THIS CLASS IS CURRENTLY OUT OF USE.
  PLEASE USE ANOTHER CLASS IF YOU WANT STUFF TO HAPPEN.
  */

  boolean isFinished;
  public PrimaryLiftCommand() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.pnuematics.changeLiftMode();
    isFinished = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

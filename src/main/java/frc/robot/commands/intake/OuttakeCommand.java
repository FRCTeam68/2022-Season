// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCommand extends CommandBase {

  public OuttakeCommand() {

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.intake.setSpeedOfIndexAndIntake(-.25, -.25, -5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

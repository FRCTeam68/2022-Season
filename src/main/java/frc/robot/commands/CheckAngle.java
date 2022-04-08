// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CheckAngle extends CommandBase {
  /** Creates a new CheckAngle. */

  private int phase = 0;

  private AdvClimb climb1 = new AdvClimb();
  private AdvancedClimb climb2 = new AdvancedClimb();

  public CheckAngle(int phase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.phase = phase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (phase == 0){
      if (Robot.climber.getAngleToGround() == 0){
        climb1.schedule(true);
      }
    }

    if (phase == 1){
      if (Robot.climber.getAngleToGround() == 0){
        climb2.schedule(true);
      }
    }

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

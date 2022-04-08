// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ChangeRPM extends CommandBase {
  /** Creates a new IndexCommand. */

  boolean isFinished = false;
  private int button = 0;

  public ChangeRPM(int button) {
    this.button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pastTarget = Robot.shooter.targetRPM;

    switch (button){
        case Constants.CONTROLLOR_MANIP_X:
            // Robot.shooter.targetRPM = 1500;
            System.out.println("Oooga booga");
            Robot.shooter.targetRPM = 1000;
            break;
        case Constants.CONTROLLOR_MANIP_SQUARE:
            // Robot.shooter.targetRPM = 2550;
            Robot.shooter.targetRPM = 1200;
            break;
        case Constants.CONTROLLOR_MANIP_TRIANGLE:
            // Robot.shooter.targetRPM = 3033;
            Robot.shooter.targetRPM = 1400;
            break;
        case Constants.CONTROLLOR_MANIP_RB:
            // Robot.shooter.targetRPM = Robot.shooter.m_calculateRPM();
            Robot.shooter.targetRPM = 0;
            break;
        default:
            Robot.shooter.targetRPM = 0;
            break;
    }

    

    if (pastTarget == Robot.shooter.targetRPM){
      Robot.shooter.targetRPM = 0;
    }

    Robot.shooter.setRPM(Robot.shooter.targetRPM);

    isFinished = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

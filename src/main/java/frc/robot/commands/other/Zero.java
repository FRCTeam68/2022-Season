/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.other;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Commandstants;

public class Zero extends CommandBase {
  
  boolean isDone = false;

  private int id = 0;

  public Zero(int id) {
    this.id = id;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (id){
      case Commandstants.GYRO_ZERO:
        RobotContainer.m_drivetrainSubsystem.zeroGyroscope();
        break;
      case Commandstants.CLIMBER_ZERO:
        Robot.climber.ResetEncoders();
        break;
    }
    
    isDone = true;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}

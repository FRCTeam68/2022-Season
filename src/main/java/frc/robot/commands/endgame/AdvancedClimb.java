// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endgame;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.parent.SequentialCommandBase;

public class AdvancedClimb extends CommandBase {
    boolean isFinished = false;

    public int phase = 0;

  public AdvancedClimb() {
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    switch (phase){
        case 0:
            firstPhase();
            break;
        case 1:
            secondPhase();
            break;
        default:
            isFinished = true;
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.setLeftSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

    private void secondPhase() {
        if ((Robot.climber.getRightEncoder()/1000) > -0.7){
            Robot.climber.setLeftPos(282563);
        }
  
        if (Robot.climber.getRightEncoder()/1000 < 280){
          Robot.climber.setLeftPos(281008);
          phase = 1;     
       }
    }

  private void firstPhase(){
      if ((Robot.climber.getLeftEncoder()/1000) < 270){
          Robot.climber.setRightPos(-282563);
      }

      if ((Robot.climber.getRightEncoder()/1000) < -279){
          Robot.climber.setLeftPos(100000);
      }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endgame;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Commandstants;

public class ManualClimb extends CommandBase {

  private boolean isFinished = false;
  private int id;
  private double pCurrentL = Robot.climber.getCurrents()[0], pCurrentR = Robot.climber.getCurrents()[1]; // Past current so we can keep track of changes in current L -> Left R -> Right p -> Past

  public ManualClimb(int id) {
    this.id = id;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Robot.climber.setLeftPos(0);
    switch (id){
      case Commandstants.R_CLIMB_UP:
        setRight(1);
        break;
      case Commandstants.R_CLIMB_DOWN:
        setRight(-1);
        break;
      case Commandstants.L_CLIMB_UP:
        setLeft(1);
        break;
      case Commandstants.L_CLIMB_DOWN:
        setLeft(-1);
        break;
    }
    isFinished = true;
  }

  @Override
  public void end(boolean interrupted) {
    if (id == Commandstants.L_CLIMB_UP || id == Commandstants.L_CLIMB_DOWN){
      Robot.climber.setLeftSpeed(0);
    } else {
    Robot.climber.setRightSpeed(0); // Make sure these guys both stop, although we might not want that. 
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  private void setLeft(int dir){
    // 1 is Up and -1 is Down
    if (shouldContinue()){ // Test is the change in current is too large
      if (dir == -1) { // Check direction of call
        
        if (Math.abs(Robot.climber.getLeftEncoder()) > 1000){ // Stops at this encoder tick. Down
          Robot.climber.setLeftSpeed(Constants.lDir * -0.2);
        } else {
          Robot.climber.setLeftSpeed(0);
        }
      } else {
        if (Math.abs(Robot.climber.getLeftEncoder()) < 280*1000){ // Also stops at this encoder tick, measured beforehand UP
          Robot.climber.setLeftSpeed(Constants.lDir * 0.2);
    
        } else {
          Robot.climber.setLeftSpeed(0);
        }
      }
    } else {
      int time = 0;
      while (time <= 10){ // Go back up for 10 cycles. Hopefully you didn't reverse the stupid direction because that would not be good.
        Robot.climber.setLeftSpeed(Constants.lDir * -0.2);
      }
      Robot.climber.setLeftSpeed(0);
      isFinished = true; // Stop entirely
    }
  
  }

  private void setRight(int dir){ // Do the same with the right because function are weird.
    if (shouldContinue()){ // Test is the change in current is too large (Might need some revamping because the starting has too high current)
      if (dir == -1) { // Check direction of call
        
        if (Math.abs(Robot.climber.getRightEncoder()) > 1000){ // Stops at this encoder tick.
          Robot.climber.setRightSpeed(Constants.rDir * -0.2);
        } else {
          Robot.climber.setRightSpeed(0);
        }
      } else {
        if (Math.abs(Robot.climber.getRightEncoder()) < 280*1000){ // Also stops at this encoder tick, measured beforehand
          Robot.climber.setRightSpeed(Constants.rDir*0.2);
    
        } else {
          Robot.climber.setRightSpeed(0);
        }
      }
    } else {
      int time = 0;
      while (time <= 10){ // Go back up for 10 cycles. Hopefully you didn't reverse the stupid direction because that would not be good.
        Robot.climber.setRightSpeed(Constants.rDir * -0.2);
      }
      Robot.climber.setRightSpeed(0);
      isFinished = true;
    }
  }

  private boolean shouldContinue(){
    // Keep track of current
    // d -> delta  c -> current;
    double[] cCurrent = Robot.climber.getCurrents(); // L is 0 R is 1 (Look at your hands from left - right)
    double dCurrentL = cCurrent[0] - pCurrentL;
    double dCurrentR = cCurrent[1] - pCurrentR;

    // Check for current and if not reverse and other stuff. Change values if we ever find out.
    if (dCurrentL < 5 && dCurrentR < 5) {
      return true;
    }

    pCurrentL = cCurrent[0]; // Set past currents to now currents.
    pCurrentR = cCurrent[1];
    return false;
  }
}

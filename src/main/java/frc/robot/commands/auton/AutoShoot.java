// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoShoot extends CommandBase {

  private boolean isHigh = false, isLow = false, toggle = false, isFinished = false;

  private double targetError = 100;

  public AutoShoot() {
  }

  @Override
  public void initialize() {
    toggle = !toggle;
    isFinished = !toggle;
  }

  @Override
  public void execute() {
    while (toggle){
    if (Robot.intake.getIntakeBeamBreak()){
      Robot.shooter.graceTime = 0;
        isLow = true;
    }else {
      isLow = false;
    }
    if (Robot.intake.getIndexBeamBreak()){
      Robot.shooter.graceTime = 0;
        isHigh = true;
        //Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
    } else {
      isHigh = false;
    }
    System.out.println(toggle);
    //Robot.intake.setSpeedOfIndexAndIntake(.5, .4, 0);

    if (isHigh) {
      if (Robot.shooter.getCurrentRPM() > (Robot.shooter.targetRPM-targetError) && 
      Robot.shooter.getCurrentRPM() < (Robot.shooter.targetRPM+targetError)){
        Robot.intake.setSpeedOfIndexAndIntake(.5, 0, 0);
      } else {
        Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
      }
    }

    if (!isLow && !isHigh){
      Robot.shooter.graceTime++;
    }

    if (isLow && !isHigh){
      
      Robot.intake.setSpeedOfIndexAndIntake(.7, .5, 0);
    }
    
    if (!(isLow || isHigh) && Robot.shooter.graceTime > Robot.shooter.gracePeroid){
     isFinished  = true;
     toggle = false;
    }
  }
    // We might nee d a variable called shooting to keep the index from fighting itself?
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
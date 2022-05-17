// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.parent.Command68;

public class TurretMove extends Command68 {

  public TurretMove(int dir) {
    // 1 is left -1 is right. These are the numbers because I say so.
    super(dir);
  }

  @Override
  public void execute() {
    if (actionId == 1){
      Robot.turret.setTurretSpeed(-.5);
    } else {
      Robot.turret.setTurretSpeed(.5);
    }
  }
}

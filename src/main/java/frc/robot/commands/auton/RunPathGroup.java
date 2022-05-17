// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.parent.SequentialCommandBase;
import frc.robot.subsystems.PathFollower;

public class RunPathGroup extends SequentialCommandBase {

  RunPath runPath = null;
  AutonCommand autonCommand = null;

  public RunPathGroup() {
    runPath = new RunPath();
    autonCommand = new AutonCommand();
    addCommands(new ParallelCommandGroup(
      autonCommand,
      new SequentialCommandGroup(
      new WaitCommand(2),
      runPath)
      ));
  }

  @Override
  public void setPathFollowers(PathFollower... paths){
    this.paths = paths;
    runPath.setPath(paths[0]);
  }
}

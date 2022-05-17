// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.ChangeIntakePos;
import frc.robot.commands.intake.IntakeCommand;


public class AutonCommand extends SequentialCommandGroup {

  public AutonCommand() {

    // Might do something nice enough for stopping the commands after a set amount of time
    // Just experimenting at the moment, but it should work hopefully.
    
    /* ParallelDeadlineGroup commands = new ParallelDeadlineGroup( new SequentialCommandGroup(
        new ChangeIntakePos(),
        new WaitCommand(.5),
        new ChangeIntakePos(),
        new IntakeCommand(),
        new WaitCommand(1),
        new AutonHighIndex(),
        new WaitCommand(1),
        new AutonIndexLow()
        ),
        new AutonShot()
        ));
     commands.setDeadline(new WaitCommand(10));  */

    addCommands(
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
        new ChangeIntakePos(),
        new ChangeIntakePos(),
        new WaitCommand(.5),
        new ChangeIntakePos(),
        new IntakeCommand(),
        new WaitCommand(1),
        new AutonHighIndex(),
        new WaitCommand(1),
        new AutonIndexLow()
        ),
        new AutonShot()
      ));
        
      
      /*
      new WaitCommand(7),
      new SequentialCommandGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
            new ShootStop(),
            new StopIndex()
          )
        )
      )
      */
      
  }
}

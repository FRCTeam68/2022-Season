// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCommand extends SequentialCommandGroup {
  /** Creates a new AutonCommand. */
  public AutonCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

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

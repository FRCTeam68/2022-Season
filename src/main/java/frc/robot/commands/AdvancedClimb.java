// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.parent.SequentialCommandBase;

public class AdvancedClimb extends SequentialCommandBase {
    /** Creates a new AutonHighIndex. */
    boolean isfinished = false;

    public int phase = 0;

    public AdvancedClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new EndgameClimbPos(0, Constants.RIGHT_ARM_MOTOR),
        new EndgameClimbPos(0, Constants.LEFT_ARM_MOTOR),
        new WaitCommand(0),
        new CheckAngle(1) // Make stuff connect and bring up
    );
    }
}

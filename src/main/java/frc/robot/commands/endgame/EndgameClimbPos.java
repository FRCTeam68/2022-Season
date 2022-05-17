// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endgame;

import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class EndgameClimbPos extends CommandBase {

    boolean isfinished = false;

    private double encoderPos;
    private int climber;

    public EndgameClimbPos(double encoderPos, int climber) {
        this.encoderPos = encoderPos;
        this.climber = climber;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(climber){
            case Constants.RIGHT_ARM_MOTOR:
                Robot.climber.setRightPos(encoderPos);
                break;
            case Constants.LEFT_ARM_MOTOR:
                Robot.climber.setLeftPos(encoderPos);
                break;

        }
       
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return isfinished;
    }   
}

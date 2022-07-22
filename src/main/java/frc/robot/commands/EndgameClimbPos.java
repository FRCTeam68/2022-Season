package frc.robot.commands;

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
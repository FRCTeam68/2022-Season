package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RunPath;
import frc.robot.commands.auton.RunPathGroup;
import frc.robot.commands.parent.SequentialCommandBase;
import frc.robot.subsystems.PathFollower;

public enum Autons {
    
    RUNPATH(new RunPathGroup());

    SequentialCommandBase command;

    Autons(SequentialCommandBase command){
        this.command = command;
    }

    public SequentialCommandBase getSequentialCommandBase(){
        return this.command;
    }

    public void setPathFollowers(PathFollower... paths){
        this.command.setPathFollowers(paths);
    }

}

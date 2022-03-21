package frc.robot.commands.parent;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PathFollower;

public abstract class SequentialCommandBase extends SequentialCommandGroup{
    
    protected PathFollower paths[];

    public void setPathFollowers(PathFollower... paths){
        this.paths = paths;
    }

    public PathFollower[] getPathFollowers(){
        return this.paths;
    }
}

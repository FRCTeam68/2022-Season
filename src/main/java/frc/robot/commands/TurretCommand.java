package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TurretCommand extends CommandBase{

    public TurretCommand(){
        addRequirements(Robot.turret);
    }
    
    
}

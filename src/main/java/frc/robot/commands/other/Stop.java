package frc.robot.commands.other;

import frc.robot.Robot;
import frc.robot.commands.Commandstants;
import frc.robot.commands.parent.Command68;

public class Stop extends Command68{

    public Stop(int id){
        super(id);
    }

    @Override
    public void execute(){
        switch(this.actionId){
            case Commandstants.TURRET_STOP:
                Robot.turret.setTurretSpeed(0);
                break;
            case Commandstants.SHOOT_STOP:
                Robot.shooter.setRPM(0);
                break;
            case Commandstants.INTAKE_STOP:
                Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
                break;
            case Commandstants.INDEX_STOP:
                Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
                break;
        }
        isFinished = true;
    }
    
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase{

    boolean isdone = false;
    public IntakeCommand() {

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if(Robot.intake.getIndexBeamBreak() && Robot.intake.getIntakeBeamBreak()){
            Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
            isdone = true;
        }
        else if(Robot.intake.getIndexBeamBreak()){
            Robot.intake.setSpeedOfIndexAndIntake(0, .25, .8);
        }
        else{
            Robot.intake.setSpeedOfIndexAndIntake(.15, .25, .8);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return isdone;
    }

}

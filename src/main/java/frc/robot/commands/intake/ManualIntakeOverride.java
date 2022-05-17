package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualIntakeOverride extends CommandBase{
    public ManualIntakeOverride() {

    }
    
      @Override
      public void initialize() {}
    
      @Override
      public void execute() {
        Robot.intake.setSpeedOfIndexAndIntake(.30, .35, .8);
        /*
        if(Robot.shooter.goodToShoot(Constants.shooterTargetRPM())){
          Robot.intake.setSpeedOfIndexAndIntake(.5, .2, 0);
        }
        else{
          Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
        }
        */
      }
    
      @Override
      public void end(boolean interrupted) {
        Robot.intake.setSpeedOfIndexAndIntake(0, 0, 0);
      }
    
      @Override
      public boolean isFinished() {
        return false;
      }
}

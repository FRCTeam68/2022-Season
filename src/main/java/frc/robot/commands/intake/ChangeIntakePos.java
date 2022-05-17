package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChangeIntakePos extends CommandBase {
  
  public boolean isFinishedCo;
  public ChangeIntakePos() {
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.pnuematics.changeIntakeMode(); 
    isFinishedCo = true;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isFinishedCo;
  }
}

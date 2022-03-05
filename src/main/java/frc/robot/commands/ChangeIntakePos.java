package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChangeIntakePos extends CommandBase {
  /**
   * Creates a new ChangeIntakePos.
   */
  public boolean isFinishedCo;
  public ChangeIntakePos() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.pnuematics.changeIntakeMode(); //Open back up once we are actually programming stuff
   //Robot.pnuematics.setSolenoidOut(Robot.pnuematics.getIntakeSolenoid());
    isFinishedCo = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedCo;
  }
}

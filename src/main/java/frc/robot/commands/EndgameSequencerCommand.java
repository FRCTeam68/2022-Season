package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class EndgameSequencerCommand extends CommandBase{
        Boolean finished = false;
        public EndgameSequencerCommand() {
          // Use addRequirements() here to declare subsystem dependencies.
          addRequirements(Robot.pnuematics);
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          Robot.pnuematics.changeSolenoidPosition(Robot.pnuematics.getEndgameSolenoid1());
          finished = true;
        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          return finished;
        
      }
    
}

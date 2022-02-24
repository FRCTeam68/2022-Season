package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IndexCommand extends CommandBase {
    
    public boolean hasBall = false; // I honestly don't really know what to do here.

    public boolean manualOverride = false;

    public IndexCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Technically this should go until 
        if ((hasBall && Robot.intake.doesTopSense()) || manualOverride){
            // Intake.getIntake().setIndexSpeed(1); //Not entirely sure what to do here.
        }
      

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}

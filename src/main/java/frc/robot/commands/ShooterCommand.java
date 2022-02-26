package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterCommand extends CommandBase{
    
    public ShooterCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
       // addRequirements(Robot.shooter);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.shooter.setSpeed(.5);
        //Robot.shooter.speedByEncoder(); //Not entirely sure what to do here.

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

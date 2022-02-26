package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase{

    public IntakeCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(Robot.intake.getIndexBeamBreak()){
            Robot.intake.setSpeedOfIndexAndIntake(0, Robot.m_robotContainer.getRightXboxManipulatorJoystickValue());
        }
        else if(Robot.intake.getIndexBeamBreak() && Robot.intake.getIntakeBeamBreak()){
            Robot.intake.setSpeedOfIndexAndIntake(0, 0);
        }
        else{
            Robot.intake.setSpeedOfIndexAndIntake(Robot.m_robotContainer.getRightXboxManipulatorJoystickValue(), Robot.m_robotContainer.getRightXboxManipulatorJoystickValue());
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

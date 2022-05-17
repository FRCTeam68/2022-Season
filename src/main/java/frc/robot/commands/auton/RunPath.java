// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PathFollower;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunPath extends CommandBase {

  PathFollower path = null;

  /** Creates a new RunPath. */
  public RunPath() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_drivetrainSubsystem.setPathPlannerFollower(path, true);
    path.resetStart();
    //Robot.m_robotContainer.m_drivetrainSubsystem.resetOdometry(path.getInitialPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_drivetrainSubsystem.driveOnPath();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrainSubsystem.driveAutonomously(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return path.isFinished();
  }

  public void setPath(PathFollower path){
    this.path = path;
  }
}

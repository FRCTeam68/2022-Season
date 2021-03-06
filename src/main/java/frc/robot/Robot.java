// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.*;


import frc.robot.commands.parent.SequentialCommandBase;
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SequentialCommandBase m_autonomousCommand;

  public static double velocity;

  public static Intake intake;
  public static Shooter shooter;
  public static Pnuematics pnuematics;
  public static RobotContainer m_robotContainer;
  public static Turret turret;
  public static Vision vision;
  public static PathFollower pathFollower;
  public static boolean isTeleop = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    pathFollower = new PathFollower("DriveStraight");
    Autons.RUNPATH.setPathFollowers(Robot.pathFollower);
    m_robotContainer = new RobotContainer();

    intake = new Intake();
    shooter = new Shooter();
    pnuematics = new Pnuematics();
    turret = new Turret();
    vision = new Vision(); 
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    isTeleop = false;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  
    // I think I'm supposed to reset encoders, but I don't know what I'm doing so I didn't
    //I also just stopped here because I don't know if I actually need to do this part.

    m_autonomousCommand = Autons.RUNPATH.getSequentialCommandBase();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    isTeleop = false;
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_drivetrainSubsystem.zeroGyroscope();
    Robot.turret.ResetEncoders();
    Robot.vision.setCameraMode(1, 1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    isTeleop = true;
    SmartDashboard.putNumber("RightStick", Robot.m_robotContainer.getRightManipulatorJoystickValue());
    SmartDashboard.putBoolean("Good to Shoot", Robot.shooter.goodToShoot(Constants.shooterTargetRPM()));
    SmartDashboard.putNumber("Limelight Distance", Robot.vision.calcDistance());
    SmartDashboard.putNumber("Wheel RPM", Robot.shooter.getWheelRPM());
    //SmartDashboard.putNumber("LeftX", Robot.m_robotContainer.getLeftX());
    //SmartDashboard.putNumber("LeftY", Robot.m_robotContainer.getLeftY());
    
    SmartDashboard.putNumber("ODMX", RobotContainer.m_drivetrainSubsystem.getPose().getX());
    SmartDashboard.putNumber("ODMY", RobotContainer.m_drivetrainSubsystem.getPose().getY());
    SmartDashboard.putNumber("Rotation", RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  
}

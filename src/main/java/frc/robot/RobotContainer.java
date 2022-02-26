 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.EndgameSequencerCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController driveController = new XboxController(Constants.CONTROLLOR_DRIVE);
  private final XboxController manipController = new XboxController(Constants.CONTROLLOR_MANIP);
  private JoystickButton manipCircle;
  private JoystickButton manipSquare;
  private JoystickButton manipX;
  private JoystickButton manipTriangle;
  private JoystickButton manipLT;
  private JoystickButton driveSelect;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(driveController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driveController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driveController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();

    // Set comands for the buttons
    driveSelect.whenPressed(new ZeroGyro());
    manipSquare.whileHeld(new ShooterCommand());
    manipSquare.whenReleased(new Zero());
    manipTriangle.whileHeld(new TurretCommand());
    manipTriangle.whenReleased(new Zero());
    manipLT.whileHeld(new EndgameSequencerCommand());
    manipLT.whenReleased(new Zero());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    driveSelect = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_SELECT);
    manipCircle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_CIRCLE);
    manipSquare = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_SQUARE);
    manipX = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_X);
    manipTriangle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_TRIANGLE);
    manipLT = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_LT);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public double getRightXboxManipulatorJoystickValue() {
    double rightAxis;
    rightAxis = manipController.getRightY();
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;

  }
}

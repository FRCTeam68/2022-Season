 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  //private final XboxController manipController = new XboxController(Constants.CONTROLLOR_MANIP)
  private JoystickButton driveY;
  private JoystickButton driveA;
  private JoystickButton driveRB;
  private JoystickButton driveLB;
  private JoystickButton manipCircle;
  private JoystickButton manipSquare;
  private JoystickButton manipX;
  private JoystickButton manipTriangle;
  private JoystickButton manipLT;
  private JoystickButton manipRT;
  private JoystickButton manipRB;
  private JoystickButton manipLB;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
            () -> -modifyAxis(driveController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driveController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driveController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    
    // Configure the button bindings
    configureButtonBindings();
    configureButtonBindings();

    // Set commands for the driver buttons
    driveA.whenPressed(new ZeroGyro());
    driveY.whenPressed(new PrimaryLiftCommand());
    driveRB.whileHeld(new TurretMoveRight());
    driveRB.whenReleased(new TurretStop());
    driveLB.whileHeld(new TurretMoveLeft());
    driveLB.whenReleased(new TurretStop());
    //set commands for the manip buttons
    manipLT.whileHeld(new IntakeCommand());
    manipLT.whenReleased(new StopIntake());
    manipRT.whileHeld(new IndexCommand());
    manipRT.whenReleased(new StopIndex());
    manipLB.whenPressed(new ChangeIntakePos());
    
    manipRB.whileHeld(new TurretCommand());
    manipRB.whenReleased(new TurretStop());
    manipRB.whileHeld(new CameraModeOn());
    manipRB.whenReleased(new CameraModeOff());
    manipRB.whileHeld(new Shoot());
    manipRB.whenReleased(new ShootStop());

    manipCircle.whileHeld(new OuttakeCommand());
    manipCircle.whenReleased(new StopIntake());

    manipSquare.whileHeld(new Shoot());
    //manipSquare.whenReleased(new Zero());

    manipX.whileHeld(new Shoot());
    //manipX.whenReleased(new ShootStop());

    manipTriangle.whileHeld(new Shoot());
   //manipTriangle.whenReleased(new ShootStop());
    
    //createSmartDashboardNumber("RPM", 0);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    driveA = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_A);
    driveY = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_Y);
    driveRB = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_RB);
    driveLB = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_LB);

    manipCircle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_CIRCLE);
    manipSquare = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_SQUARE);
    manipX = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_X);
    manipTriangle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_TRIANGLE);
    manipLT = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_LT);
    manipLB = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_LB);
    manipRB = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_RB);
    manipRT = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_RT);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   //return new InstantCommand();
  //   //return new AutonCommand();
  //   return new RunPath();
  // }

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

  public double getLeftX(){
    double leftXAxis;
    leftXAxis = driveController.getLeftX();
    return leftXAxis;
  }
  public double getLeftY(){
    double leftYAxis;
    leftYAxis = driveController.getLeftY();
    return leftYAxis;
  }
  public double getRightManipulatorJoystickValue() {
    double rightAxis;
    
    rightAxis = modifyAxis(manipController.getRawAxis(2));
    
    return rightAxis;
  }

  public boolean getManipRB() {
		boolean buttonPressed = false;
		if (manipRB.get()) {
			buttonPressed = true;
		}
		return buttonPressed;
	}

  public boolean getManipX(){
    boolean buttonPressed = false;
    if(manipX.get()){
      buttonPressed = true;
    }
    return buttonPressed;
  }

  public boolean getManipSquare(){
    boolean buttonPressed = false;
    if(manipSquare.get()){
      buttonPressed = true;
    }
    return buttonPressed;
  }

  public boolean getManipTriangle(){
    boolean buttonPressed = false;
    if(manipTriangle.get()){
      buttonPressed = true;
    }
    return buttonPressed;
  }

  public static double createSmartDashboardNumber(String key, double defValue) {

    // See if already on dashboard, and if so, fetch current value
    double value = SmartDashboard.getNumber(key, defValue);
  
    // Make sure value is on dashboard, puts back current value if already set
    // otherwise puts back default value
    SmartDashboard.putNumber(key, value);
  
    return value;
  }
  public double getRPM(){
    return SmartDashboard.getNumber("RPM", 0);
  }
}

 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Commandstants;
import frc.robot.commands.auton.AutoShoot;
import frc.robot.commands.drivetrain.DefaultDriveCommand;
import frc.robot.commands.endgame.AutoClimb;
import frc.robot.commands.endgame.ManualClimb;
import frc.robot.commands.intake.ChangeIntakePos;
import frc.robot.commands.intake.IndexCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ManualIntakeOverride;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.other.CameraModeOff;
import frc.robot.commands.other.CameraModeOn;
import frc.robot.commands.other.Stop;
import frc.robot.commands.other.Zero;
import frc.robot.commands.shooter.ChangeRPM;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.TurretCommand;
import frc.robot.commands.shooter.TurretMove;
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
  private final XboxController whatController = new XboxController(Constants.CONTROLLOR_QUESTION);
  //private final XboxController manipController = new XboxController(Constants.CONTROLLOR_MANIP)
  private JoystickButton driveY;
  private JoystickButton driveA;
  private JoystickButton driveX;
  private JoystickButton driveB;
  private JoystickButton driveRB;
  private JoystickButton driveLB;
  private JoystickButton driveStart;
  private JoystickButton driveSelect;
  private JoystickButton manipCircle;
  private JoystickButton manipSquare;
  private JoystickButton manipX;
  private JoystickButton manipTriangle;
  private JoystickButton manipLT;
  private JoystickButton manipRT;
  private JoystickButton manipRB;
  private JoystickButton manipLB;
  private JoystickButton manipPad;
  private JoystickButton manipShare;
  private JoystickButton whatY;
  private JoystickButton whatA;
  private JoystickButton whatX;
  private JoystickButton whatB;
  private JoystickButton whatRB;
  

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


    // Set commands for the driver buttons
    driverControlBinds();
    manipulatorControlBinds();

    //set commands for the manip buttons
    
  // For use when we have nothing else to do. For now look at this bear: ʕ – ㉨ – ʔ
  //  whatRB.whileHeld(new AdvClimb());  
    
    //createSmartDashboardNumber("RPM", 0);
    
  }

  private void driverControlBinds(){
    driveStart.whenPressed(new Zero(Commandstants.GYRO_ZERO));
    driveRB.whileHeld(new TurretMove(-1)); // Right
    driveRB.whenReleased(new Stop(Commandstants.TURRET_STOP));
    driveLB.whileHeld(new TurretMove(1)); //Left
    driveLB.whenReleased(new Stop(Commandstants.TURRET_STOP));

    // whatY.whileHeld(new ManualClimb(Commandstants.R_CLIMB_UP));
    // whatB.whileHeld(new ManualClimb(Commandstants.R_CLIMB_DOWN));
    // whatX.whileHeld(new ManualClimb(Commandstants.L_CLIMB_UP));
    // whatA.whileHeld(new ManualClimb(Commandstants.L_CLIMB_DOWN));
    
    driveSelect.whenPressed(new Zero(Commandstants.CLIMBER_ZERO));

    // driveY.whenPressed(new AutoClimb(Commandstants.R_CLIMB_UP));
    // driveB.whenPressed(new AutoClimb(Commandstants.R_CLIMB_DOWN));
    // driveX.whenPressed(new AutoClimb(Commandstants.L_CLIMB_UP));
    // driveA.whenPressed(new AutoClimb(Commandstants.L_CLIMB_DOWN));
  }

  private void manipulatorControlBinds(){
    manipLT.whileHeld(new IntakeCommand());
    manipLT.whenReleased(new Stop(Commandstants.INTAKE_STOP));
    manipRT.whileHeld(new IndexCommand());
    manipRT.whenReleased(new Stop(Commandstants.INDEX_STOP));
    manipLB.whenPressed(new ChangeIntakePos());
    
    manipRB.whileHeld(new TurretCommand());
    manipRB.whenReleased(new Stop(Commandstants.TURRET_STOP));
    manipRB.whileHeld(new CameraModeOn());
    manipRB.whenReleased(new CameraModeOff());
    manipRB.whileHeld(new Shoot());
    manipRB.whenReleased(new Stop(Commandstants.SHOOT_STOP));

    manipCircle.whileHeld(new OuttakeCommand());
    manipCircle.whenReleased(new Stop(Commandstants.INTAKE_STOP));

    manipShare.whileHeld(new ManualIntakeOverride());

    // Is whenPressed on release? Might need to test a bit, but then also change if needed or somn
    manipSquare.whenPressed(new ChangeRPM(Constants.CONTROLLOR_MANIP_SQUARE));
    //manipSquare.whenReleased(new Zero());

    manipX.whenPressed(new ChangeRPM(Constants.CONTROLLOR_MANIP_X));
    //manipX.whenReleased(new ShootStop());

    manipTriangle.whenPressed(new ChangeRPM(Constants.CONTROLLOR_MANIP_TRIANGLE));
   //manipTriangle.whenReleased(new ShootStop());

   manipPad.whenPressed(new AutoShoot());
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
    driveB = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_B);
    driveX = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_X);
    driveY = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_Y);
    driveRB = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_RB);
    driveLB = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_LB);
    driveSelect = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_SELECT);
    driveStart = new JoystickButton(driveController, Constants.CONTROLLOR_DRIVE_START);

    manipCircle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_CIRCLE);
    manipSquare = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_SQUARE);
    manipX = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_X);
    manipTriangle = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_TRIANGLE);
    manipLT = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_LT);
    manipLB = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_LB);
    manipRB = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_RB);
    manipRT = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_RT);
    manipPad = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_START);
    manipShare = new JoystickButton(manipController, Constants.CONTROLLOR_MANIP_SELECT);

    whatA = new JoystickButton(whatController, Constants.CONTROLLOR_MANIP_X);
    whatB = new JoystickButton(whatController, Constants.CONTROLLOR_MANIP_CIRCLE);
    whatX = new JoystickButton(whatController, Constants.CONTROLLOR_MANIP_SQUARE);
    whatY = new JoystickButton(whatController, Constants.CONTROLLOR_MANIP_TRIANGLE);
    whatRB= new JoystickButton(whatController, Constants.CONTROLLOR_MANIP_RB);
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

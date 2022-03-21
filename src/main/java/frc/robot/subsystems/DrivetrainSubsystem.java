// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 10.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  
  Rotation2d gyroOffset = new Rotation2d(0);
  
 
  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  //private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final Mk3ModuleConfiguration currentLimit;

  private double frontLeft_stateAngle = 0.0,
			frontRight_stateAngle = 0.0,
			backLeft_stateAngle = 0.0,
			backRight_stateAngle = 0.0;

      PIDController xPID = new PIDController(4,0,0);
	PIDController yPID = new PIDController(4,0,0);
	PIDController rotPID = new PIDController(8,0,0);

	PathController pathStateController = new PathController(xPID, yPID, rotPID);

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(Constants.kDriveKinematics, m_navx.getRotation2d());
      
      Pose2d targetPose;

      public double target = (getGyroscopeRotation().getDegrees());

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    currentLimit = new Mk3ModuleConfiguration();
    currentLimit.setDriveCurrentLimit(30.0);
    currentLimit.setSteerCurrentLimit(30.0);
    
    

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.

    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            //output current state of the module
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0),
                //module congig to current limit 
            currentLimit, 
                //this can be either STANDARD or FAST dpending on gear config
            Mk3SwerveModuleHelper.GearRatio.FAST, 
            FRONT_LEFT_MODULE_DRIVE_MOTOR, 
            FRONT_LEFT_MODULE_STEER_MOTOR, 
            FRONT_LEFT_MODULE_STEER_ENCODER, 
            FRONT_LEFT_MODULE_STEER_OFFSET
            );
   
   

    // We will do the same for the other modules
   
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0), 
            currentLimit, 
            Mk3SwerveModuleHelper.GearRatio.FAST, 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
            );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0), 
            currentLimit, 
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
            );
 
    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0), 
            currentLimit, 
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
            );

  }

  public SwerveModuleState getFLState(){
        return new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle()));
  }
  public SwerveModuleState getFRState(){
        return new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle()));
  }
  public SwerveModuleState getBLState(){
        return new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle()));
  }
  public SwerveModuleState getBRState(){
        return new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle()));
  }
  public void setModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);

      m_frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[0].angle.getRadians());
      m_frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[1].angle.getRadians());
      m_backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
      m_backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());

      desiredStates[0].speedMetersPerSecond = Math.abs(m_frontLeftModule.getDriveVelocity());
      desiredStates[1].speedMetersPerSecond = Math.abs(m_frontRightModule.getDriveVelocity());
      desiredStates[2].speedMetersPerSecond = Math.abs(m_backLeftModule.getDriveVelocity());
      desiredStates[3].speedMetersPerSecond = Math.abs(m_backRightModule.getDriveVelocity());
      m_odometry.update(getGyroscopeRotation(), desiredStates);

      SmartDashboard.putNumber("Current X", getPose().getX()); 
      SmartDashboard.putNumber("Current Y", getPose().getY()); 
      SmartDashboard.putNumber("Auto Angle", getPose().getRotation().getDegrees()); 
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public Pose2d getPose() {
        return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Translation2d position){
    getSwerveDriveOdometry().resetPosition(new Pose2d(position.getX(), position.getY(), getGyroscopeRotation()), getGyroscopeRotation());
}

  public double getHeading() {
        return m_navx.getRotation2d().getDegrees();
  }
  public void zeroGyroscope() {
    m_navx.zeroYaw();
    gyroOffset = new Rotation2d(0);
  }
  
  public Rotation2d getGyroscopeRotation() {
    // if (m_navx.isMagnetometerCalibrated()) {
    //  // We will only get valid fused headings if the magnetometer is calibrated
    //   return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    // }
   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return m_navx.getRotation2d().plus(gyroOffset);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
  

  @Override
  public void periodic() {
    SwerveModuleState[] states = Constants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    m_odometry.update(
        new Rotation2d(getGyroscopeRotation().getRadians()),
        getFLState(),
        getFRState(),
        getBLState(),
        getBRState()
        );
  }
  protected void setPathStateController(PathController psc){
    this.pathStateController = psc;
}

public PathController getPathStateController() {
    return pathStateController;
}

public void setPathPlannerFollower(PathFollower ppf, boolean setInitialPosition){
    this.getPathStateController().setPathPlannerFollower(ppf);
    if(setInitialPosition){
        setInitalPoseFromFirstPathPlannerFollower(ppf);
    }
}

public void setInitalPoseFromFirstPathPlannerFollower(PathFollower ppf){
    gyroOffset = ppf.getInitialHolonomic().minus(getGyroscopeRotation());
    resetOdometry(ppf.getInitialPosition());
    System.out.println("Initial Translation of Path (should match following odometry: " + ppf.getInitialPosition().toString());
    System.out.println("Initial Odometry Set to: " + getSwerveDriveOdometry().getPoseMeters().toString());
}

public SwerveDriveKinematics getSwerveDriveKinematics() {
    return Constants.kDriveKinematics;
}

public SwerveDriveOdometry getSwerveDriveOdometry(){
    return m_odometry;
}
  public void driveAutonomously(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		SwerveModuleState[] swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
						xSpeed, ySpeed, rot, getGyroscopeRotation())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
		if (Math.abs(swerveModuleStates[0].speedMetersPerSecond) + Math.abs(swerveModuleStates[1].speedMetersPerSecond)
				+ Math.abs(swerveModuleStates[2].speedMetersPerSecond)
				+ Math.abs(swerveModuleStates[3].speedMetersPerSecond) > 0.001) {
			frontLeft_stateAngle = swerveModuleStates[0].angle.getRadians();
			frontRight_stateAngle = swerveModuleStates[1].angle.getRadians();
			backLeft_stateAngle = swerveModuleStates[2].angle.getRadians();
			backRight_stateAngle = swerveModuleStates[3].angle.getRadians();
		}
		m_frontLeftModule.set(swerveModuleStates[0].speedMetersPerSecond
						/ Constants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				frontLeft_stateAngle);
		m_frontRightModule.set(swerveModuleStates[1].speedMetersPerSecond
						/ Constants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				frontRight_stateAngle);
		m_backLeftModule.set(swerveModuleStates[2].speedMetersPerSecond
						/ Constants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				backLeft_stateAngle);
		m_backRightModule.set(swerveModuleStates[3].speedMetersPerSecond
						/ Constants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				backRight_stateAngle);
	}

	public void driveOnPath() {
			DriveVelocities velocities = this.getPathStateController()
					.getVelocitiesAtCurrentState(this.getSwerveDriveOdometry(), this.getGyroscopeRotation());

			Translation2d currentPosition = getSwerveDriveOdometry().getPoseMeters().getTranslation();
			// System.out.println(String.format("Current Odometry [ X: %.2f Y:%.2f ] Heading [ Rot (radians): %.2f ]", currentPosition.getX(), currentPosition.getY(), getGyroHeading().getRadians()));
			// System.out.println("Current Velocity Calculations: " + velocities.toString());
			driveAutonomously(velocities.getXVel(), velocities.getYVel(), velocities.getRotVel(), true);
	}

	
  public void driveForward(double speed) {
      m_frontLeftModule.set(speed, 0);
      m_frontRightModule.set(speed, 0);
      m_backLeftModule.set(speed, 0);
      m_backRightModule.set(speed, 0);
  }

  public void driveBackward(double speed) {
    m_frontLeftModule.set(-1*speed, 0);
    m_frontRightModule.set(-1*speed, 0);
    m_backLeftModule.set(-1*speed, 0);
    m_backRightModule.set(-1*speed, 0);
  }

  public void rotate(double rotationSpeed) {
      drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
              0, // translation x supplier is 0
              0, // translation y supplier is 0
              rotationSpeed, // we only want the robot to rotate, so this value is nonzero
              new Rotation2d(0) // i dont know why we need this line...
              )
      );
  }
  
	public void neutral() {
		drive(m_chassisSpeeds);
	}

	
	public boolean abort() {
		drive(m_chassisSpeeds);
		return true;
	}
}


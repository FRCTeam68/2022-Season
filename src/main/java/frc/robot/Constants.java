// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Robot;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 20.5; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 26.5; // FIXME Measure and set wheelbase

    //public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 18; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(215.0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 15; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(288.0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 17; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(284.0-(27-0.125)); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-115.0+171.5); // FIXME Measure and set back right steer offset

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
          SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;

    public static final double MAX_AUTON_VELOCITY_METERS_PER_SECOND = 3;
    public static final double AUTON_TO_MAX_VELOCITY_RATIO = MAX_AUTON_VELOCITY_METERS_PER_SECOND/MAX_VELOCITY_METERS_PER_SECOND;

    public static final double MAX_AUTON_VOLTAGE = 12;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 3.0;
    

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
        //front left
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
        //front right
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
        //back left
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
        //back right
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2));

        public static final double kMaxSpeedMetersPerSecond = 3.1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 5; 
        public static final double kDXController = 0.0;
        public static final double kPYController = 5;
        public static final double kDYController = 0;
        public static final double kPThetaController = 5;
        public static final double kDThetaController = 0.0;
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    //Shooter Constants Below Here
    public static final int LEFT_SHOOTER_MOTOR = 9;
    public static final int RIGHT_SHOOTER_MOTOR = 10;
    public static final int AUTON_SHOT = 2646;
    public static final double shooterTargetRPM(){
        double RPM;
        if(Robot.m_robotContainer.getManipX()){
            RPM = 1500;
        }
        else if(Robot.m_robotContainer.getManipSquare()){
            RPM = 2550;
        }
        else if(Robot.m_robotContainer.getManipTriangle()){
            RPM = 3033;
        }
        else if(Robot.m_robotContainer.getManipRB()){
            RPM = Robot.shooter.m_calculateRPM();
        }
        else{
            RPM = 0;
        }
        return RPM;
    }
    
    public static double RELEASE_ANGLE = 60.0; //degrees from horizontal
    public static double THETA = Math.toRadians(RELEASE_ANGLE); 
    public static double COMPRESSED_RADIUS = 4; //in
    public static double FLYWHEEL_RADIUS = 2.0; //in
    public static double SLIPPERINESS = 0.94;
    //Turret Constants Below Here
    public static final int TURRET_MOTOR = 11;

    public static final int TURRET_GEAR = 140;
    public static final int TURRET_SPUR = 10;
    public static final int TURRET_BOX = 45;

    //Endgame Constants Below Here

    //Pneumatics Constants below here
    public static final int ENDGAME_PRIMARY_LIFT_A = 12;
    public static final int ENDGAME_PRIMARY_LIFT_B = 3;
    public static final int ENDGAME_PRIMARY_CLAMP_A = 0;
    public static final int ENDGAME_PRIMARY_CLAMP_B = 0;
    public static final int ENDGAME_SECONDARY_LIFT_A = 0;
    public static final int ENDGAME_SECONDARY_LIFT_B = 0;
    public static final int ENDGAME_SECONDARY_CLAMP_A = 0;
    public static final int ENDGAME_SECONDARY_CLAMP_B = 0;
    public static final int INTAKE_PCM_A = 2;
    public static final int INTAKE_PCM_B = 13;
    public static final int AIR_PUMP_CAN = 19;
    //Intake Constantts Below Here
    public static final int INTAKE_MOTOR = 12;
    public static final int INDEX_MOTOR_HIGH = 14;
    public static final int INDEX_MOTOR_LOW = 13;
    public static final int INTAKE_SENSOR = 0;
    public static final int INDEX_SENSOR = 1;
    //Controller Constants Below Here
    public static final int CONTROLLOR_DRIVE = 0;
    public static final int FLIGHT_CONTROLLOR_AONE = 1;
    public static final int FLIGHT_CONTROLLOR_BZERO = 2;
    public static final int FLIGHT_CONTROLLOR_BTWO = 3;
    public static final int FLIGHT_CONTROLLOR_CZERO = 4;
    public static final int FLIGHT_CONTROLLOR_CTWO = 5;
    public static final int FLIGHT_CONTROLLOR_DZERO = 6;
    public static final int FLIGHT_CONTROLLOR_DTWO = 7;
    public static final int FLIGHT_CONTROLLOR_FTWO = 8;
    public static final int FLIGHT_CONTROLLOR_FZERO = 9;
    public static final int FLIGHT_CONTROLLOR_GTWO = 10;
    public static final int FLIGHT_CONTROLLOR_GZERO = 11;
    public static final int FLIGHT_CONTROLLOR_HONE = 12;
    public static final int FLIGHT_CONTROLLOR_I = 13;
    public static final int FLIGHT_CONTROLLOR_RESET = 14;
    public static final int FLIGHT_CONTROLLOR_CANCEL = 15;
    public static final int FLIGHT_CONTROLLOR_SELECT = 16;

    public static final int CONTROLLOR_MANIP = 1;
    public static final int CONTROLLOR_MANIP_SQUARE = 1;
    public static final int CONTROLLOR_MANIP_X = 2;
    public static final int CONTROLLOR_MANIP_CIRCLE = 3;
    public static final int CONTROLLOR_MANIP_TRIANGLE = 4;
    public static final int CONTROLLOR_MANIP_LB = 5;
    public static final int CONTROLLOR_MANIP_RB = 6;
    public static final int CONTROLLOR_MANIP_LT = 7;
    public static final int CONTROLLOR_MANIP_RT = 8;
    public static final int CONTROLLOR_MANIP_SELECT = 9;
    public static final int CONTROLLOR_MANIP_START = 10;
    public static final int CONTROLLOR_MANIP_LS = 11;
    public static final int CONTROLLOR_MANIP_RS = 12;
    public static final int CONTROLLOR_MANIP_PS = 13;
    public static final int CONTROLLOR_MANIP_PAD = 14;

    public static final int CONTROLLOR_DRIVE_A = 1;
    public static final int CONTROLLOR_DRIVE_B = 2;
    public static final int CONTROLLOR_DRIVE_X = 3;
    public static final int CONTROLLOR_DRIVE_Y = 4;
    public static final int CONTROLLOR_DRIVE_LB = 5;
    public static final int CONTROLLOR_DRIVE_RB = 6;
    public static final int CONTROLLOR_DRIVE_SELECT = 7;
    public static final int CONTROLLOR_DRIVE_START = 8;
    public static final int CONTROLLOR_DRIVE_LS = 9;
    public static final int CONTROLLOR_DRIVE_RS = 10;
    
    // Aircontroller stuff

    // public static final int CONTROLLER_LEFTX = 0;
    // public static final int CONTROLLER_LEFTY = 1;

    // public static final int CONTROLLER_RIGHTX = 3;
    // public static final int CONTROLLER_RIGHTY = 4;

    /*
    CONTROLLOR_MANIP_A = 1;
    public static final int CONTROLLOR_MANIP_B = 2;
    public static final int CONTROLLOR_MANIP_X = 3;
    public static final int CONTROLLOR_MANIP_Y = 4;
    public static final int CONTROLLOR_MANIP_LB = 5;
    public static final int CONTROLLOR_MANIP_RB = 6;
    public static final int CONTROLLOR_MANIP_SELECT = 7;
    public static final int CONTROLLOR_MANIP_START = 8;
    public static final int CONTROLLOR_MANIP_LS = 9;
    public static final int CONTROLLOR_MANIP_RS = 10;
    */
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
=======
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

>>>>>>> Comp
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

<<<<<<< HEAD
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(25.225); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(147.744); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(52.910); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(163.559); // FIXME Measure and set back right steer offset

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
    
    
=======
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
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(284.0); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-115.0); // FIXME Measure and set back right steer offset
>>>>>>> Comp

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
    public static final int CONTROLLOR_MANIP = 1;
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
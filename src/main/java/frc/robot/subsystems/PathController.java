// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/** Add your docs here. */
public class PathController {
    PathFollower pathPlannerFollower;
    PIDController x_pid, y_pid, rot_pid;

    public PathController(PIDController x_pid, PIDController y_pid, PIDController rot_pid){
        this.x_pid = x_pid;
        this.y_pid = y_pid;
        this.rot_pid = rot_pid;
    }

    public void setPathPlannerFollower(PathFollower pathPlannerFollower){
        this.pathPlannerFollower = pathPlannerFollower;
        resetPIDs();
    }

    public PathFollower getPathPlannerFollower(){
        return this.pathPlannerFollower;
    }

    private void resetPIDs(){
        x_pid.reset();
        y_pid.reset();
        rot_pid.reset();
    }

    public DriveVelocities getVelocitiesAtCurrentState(SwerveDriveOdometry odometry, Rotation2d currentOrientation){
        PathPlannerTrajectory.PathPlannerState pathState = this.pathPlannerFollower.getCurrentState();
        double fwd_back_position = pathState.poseMeters.getX(); //going down field, closer or farther from driver station
        double left_right_position = pathState.poseMeters.getY(); //side to side, parallel with driver station wall
        double angular_velocity = pathState.angularVelocity.getRadians();
        Rotation2d targetHolonomicHeading = pathState.holonomicRotation; //the orientation of the robot
        Rotation2d targetHeading = pathState.poseMeters.getRotation();  //the direction the robot should move in
        double x_velocity = pathState.velocityMetersPerSecond * targetHeading.getCos(); //meters per second down field
        double y_velocity = pathState.velocityMetersPerSecond * targetHeading.getSin(); //meters per second parallel to field

        double currentX = odometry.getPoseMeters().getX();
        double currentY = odometry.getPoseMeters().getY();

        // System.out.println(String.format("Current Commands Pos and Vel [ X(pos): %.2f Y(pos): %.2f ] [ X(vel): %.2f Y(vel): %.2f ]", fwd_back_position, left_right_position, x_velocity, y_velocity));
         //System.out.println(String.format("Current Commands Orientation [ Heading(pos): %.2f Heading(vel): %.2f ]", targetHolonomicHeading.getRadians(), angular_velocity));



        double output_x_vel = x_velocity + x_pid.calculate(currentX, fwd_back_position);
        double output_y_vel = y_velocity + y_pid.calculate(currentY, left_right_position);
        // TODO validate that the error below is calculated correctly, maybe plus?
        Rotation2d orientationError = targetHolonomicHeading.minus(currentOrientation); 
        //System.out.println(String.format("ERRORS: [%.2f,%.2f,%.2f]", fwd_back_position-currentX,left_right_position-currentY,orientationError.getRadians()));
        double output_rot_vel = angular_velocity + rot_pid.calculate(-orientationError.getRadians(), 0.0);

        return new DriveVelocities(output_x_vel, output_y_vel, output_rot_vel);
    }
}

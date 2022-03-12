// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class DriveVelocities {
    double xVel, yVel, rotVel;
    
    public DriveVelocities(double xVel, double yVel, double rotVel){
        this.xVel = xVel;
        this.yVel = yVel;
        this.rotVel = rotVel;
    }

    public double getXVel() {
        return xVel;
    }

    public double getYVel() {
        return yVel;
    }

    public double getRotVel() {
        return rotVel;
    }

    public String toString(){
        return String.format("Velocities[ X:%.2f Y:%.2f Rot:%.2f", xVel, yVel, rotVel);
    }
}


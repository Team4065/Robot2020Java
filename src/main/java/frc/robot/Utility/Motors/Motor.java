/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

/**
 * Add your docs here.
 */
public class Motor {
    public enum ControlMode{
        Percent,
        Velocity,
        Position
    };

    double target;
    /**
     * 
     * @param value
     * @param controlMode
     */
    public void set(double value, ControlMode controlMode){

    }

    /**
     * This sets the percent output of the motor
     * @param value The percent output ranging from -1 to 1
     */
    public void set(double value){

    }

    /**
     * Returns the number of rotations that a motor has undergone.
     * Returns NaN if no encoder is setup.
     * @return
     */
    public double getRotations(){
        return Double.NaN;
    }
}

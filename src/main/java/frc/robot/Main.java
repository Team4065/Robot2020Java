/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
The differential drive subsystems have a simulation mode that disables motor output on CAN (it outputs on PWM).
It is ment to be used to see motor output on the simulation GUI.
Any new differential drive subsystems should implement this feature (view TalonSRX_Drivetrain to learn how).
New differential drive subsystems will need new code for motor creation, motor configuration, motor output, and PID configuration.
*/

/**
 * TODO
 * Make more differential drivetrain commands 
 *    -arcade drive- done
 * 
 * Make manipulator Subsystem
 * 
 * Make manipulator mover Subsystem
 *    -implement motors
 *    -better organize variables
 *        -anchor should not be part of the segments
 *        -target should not be part of the segments
 *    -allow inverse kinematics targets for multiple segment
 *        -temporary targets for parent segments and the main target overall
 *        -these other targets are meant to control the path of the segments not the final position
 *    -implement the getting of the measured state
 *    -change all instances of worldspace to robotspace
 * 
 * Add support for max velocity and acceleration for TalonSRX_Drivetrain
 * 
 * Ensure that no functions in Vector3 require unit vectors but do not have garunteed unit vector usage.
 * 
 * Make a unity testing branch for github
 * 
 * Make unity testing tools
 *  -it is no longer random bits of code
 * 
 * Add variable length to inverse kinematics
 * 
 * Add proper documentation for inverse kinematics
 *    -change all instances of worldspace to robotspace
 * 
 * 
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

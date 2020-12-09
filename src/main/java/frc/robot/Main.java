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
 * 
 * Make a shuffle board interface
 * 
 * Make more differential drivetrain commands 
 *    -arcade drive- done
 * 
 * Make manipulator Subsystem
 * 
 * Make a recording mode for the manipulator mover
 *    -test
 * 
 * 
 * Add support for max velocity and acceleration for TalonSRX_Drivetrain
 * 
 * Make a unity testing branch for github
 * 
 * Make unity testing tools
 *  -make a dedicated utility class
 * 
 * Make tests to ensure the code actually works on a robot
 * 
 * Ramsete might spin because setting the voltage might be affected by the motor controller inversion setting
 * 
 * test drivetrain commands to ensure everything is inverted properly
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

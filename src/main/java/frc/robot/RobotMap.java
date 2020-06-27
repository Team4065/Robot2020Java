/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This is where the robot and the controller inputs are configured
 * CAN IDs must not repeat or else it will result in unexpected behaviors
 */
public class RobotMap {

    //Simulation control
    public static final boolean IS_SIMULATION_RUNNING = true;

    //Drivetrain
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MIN = 1;//CAN ID
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MAX = 3;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MIN = 4;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MAX = 6;//CAN ID

    //Controls
    public static final int CONTROLS_MAIN_CONTROLLER_ID = 0;
}

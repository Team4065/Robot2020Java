/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.ExtraMath.Vector3;

/**
 * This is where the robot and the controller inputs are configured
 * CAN IDs must not repeat or else it will result in unexpected behaviors
 */
public class RobotMap {
    public enum MotorType {
        PWM,
        CANSparkMax,
        TalonSRX
    };

    //Simulation control
    public static final boolean IS_SIMULATION_RUNNING = true;//when simulation is on all CAN motors should get a PWM counterpart with the same ID

    //Drivetrain
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MIN = 1;//CAN ID
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MAX = 3;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MIN = 4;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MAX = 6;//CAN ID

    //Controls
    public static final int CONTROLS_MAIN_CONTROLLER_ID = 0;

    //Program info
    /**
     * The delay between updates in seconds (20ms)
     */
    public static final double DELTA_TIME = 0.02;

    //Manipulator Mover
    public static final double MANIPULATOR_MOVER_ACCURACY_TOLERANCE = 0.1;
    public static final MotorType MANIPULATOR_MOVER_MOTOR_TYPE = MotorType.PWM;
    public static final int[][] MANIPULATOR_MOVER_MOTOR_IDS = new int[][]{//ID type depends on motor type (PWM)
        {7}, {8}, {9}, {10}, {11}
    };// ^    ^    ^    ^^  each sub array is a segment the numbers in the subarrays are the motor IDs for each segment
    public static final int[][] MANIPULATOR_MOVER_ENCODER_CHANNELS = new int[][]{//This variable is not needed with CAN based encoders (Digital Input)
        {0, 1}, {2, 3}, {4, 5}, {6, 7}, {8, 9}
    };
    public static final double[] MANIPULATOR_MOVER_LENGTHS = new double[]{
        2.0/5, 2.0/5, 2.0/5, 2.0/5, 2.0/5
    };
    public static final Vector3[] MANIPULATOR_MOVER_AXES = new Vector3[]{
        Vector3.forward, Vector3.right, Vector3.forward, Vector3.right, Vector3.forward
    };
    public static final double[] MANIPULATOR_MOVER_MIN_ANGLES = new double[]{
        -90, -90, -90, -90, -90
    };
    public static final double[] MANIPULATOR_MOVER_MAX_ANGLES = new double[]{
        90, 90, 90, 90, 90
    };
    public static final boolean[][] MANIPULATOR_MOVER_MOTOR_INVERSIONS = new boolean[][]{
        {false}, {false}, {false}, {false}, {false}
    };
}
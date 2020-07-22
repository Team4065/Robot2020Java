/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.ExtraMath.Vector3;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Utility.Motors.*;
import frc.robot.subsystems.manipulator_mover.ManipulatorMoverSegment;

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
    /**
     * This is the maximum PID error.
     * It can influence the speed of the robot's movements.
     * This is not how the speed of the robot's movements is supposed to be controlled.
     * This is a limiter to prevent the robot from breaking.
     */
    public static final double MANIPULATOR_MOVER_MAXIMUM_ERROR = 1;
    public static final ManipulatorMoverSegment[] MANIPULATOR_MOVER_SEGMENTS = new ManipulatorMoverSegment[]{
        //new ManipulatorMoverSegment(1, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(10, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent)//,
        new ManipulatorMoverSegment(2.0/3, Vector3.forward, -90, 90, new Motor[]{new PWM_Motor(10, 0, 1, 360)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.0/3, Vector3.right, -90, 90, new Motor[]{new PWM_Motor(11, 2, 3, 360)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.0/3, Vector3.forward, -90, 90, new Motor[]{new PWM_Motor(12, 4, 5, 360)}, Motor.ControlMode.Percent),
    };
}
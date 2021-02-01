/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Utility.Vector3;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Utility.Motors.*;
import frc.robot.subsystems.manipulator_mover.ManipulatorMoverSegment;

/**
 * This is where the robot and the controller inputs are configured CAN IDs must
 * not repeat or else it will result in unexpected behaviors
 */
public class Constants {
    public enum MotorType {
        PWM, CANSparkMax, TalonSRX
    };

    //Controls
    public static final Joystick mainController = new Joystick(0);

    //Sensors
    public static final boolean GYRO_REVERSED = false;
    
    //Program info
    public static final double DELTA_TIME = 0.02;//The delay between updates in seconds (20ms)
    public static final boolean IS_SIMULATION_RUNNING = false;//when simulation is on all CAN motors should get a PWM counterpart with the same ID

    // Robot info (in meters)
    // the Pathweaver will want some of this information
    public static final double ROBOT_TRACKWIDTH = 0.13334319315973552911;//Pathweaver: wheel base
    public static final double ROBOT_WHEEL_DIAMETER = 0.1524;
    public static final double ROBOT_WHEEL_RADIUS = 0.0762;
    public static final double ROBOT_MAX_SPEED = 3;// set to something below the free-floating wheel speed
    public static final double ROBOT_MAX_ACCELERATION = 3;// not very important due to voltage limiting (but Pathweaver might not be using the voltage limiting)

    // Robot Characterization 
    // info on how to find this data: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
    public static final double KS_VOLTS = 0.859;// ks
    public static final double KV_VOLT_SECONDS_PER_METER = 0.0215;// kv
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.00209;// ka
    public static final double KP_DRIVE_VEL = 2.29e-71;// kp
    public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(
            ROBOT_TRACKWIDTH);

    // Ramsete
    public static final double RAMSETE_B = 2;// recommended value that should work for most robots (To learn how to tune goto: https://docs.wpilib.org/en/stable/docs/software/advanced-control/trajectories/ramsete.html#constructing-the-ramsete-controller-object)
    public static final double RAMSETE_ZETA = 0.7;// recommended value that should work for most robots (To learn how to tune goto: https://docs.wpilib.org/en/stable/docs/software/advanced-control/trajectories/ramsete.html#constructing-the-ramsete-controller-object)
    public static final double RAMSETE_MAX_VOLTAGE = 10;//10 volts is the maximum to compensate for fluctuating voltages
    //do not touch the rest of the Ramsete variables they only implement the other variables
    public static final DifferentialDriveVoltageConstraint RAMSETE_AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
            KS_VOLTS,
            KV_VOLT_SECONDS_PER_METER,
            KA_VOLT_SECONDS_SQUARED_PER_METER
        ),
        DIFFERENTIAL_DRIVE_KINEMATICS,
        RAMSETE_MAX_VOLTAGE);
    public static final TrajectoryConfig RAMSETE_TRAJECTORY_CONFIG = new TrajectoryConfig(
        ROBOT_MAX_SPEED, 
        ROBOT_MAX_ACCELERATION
        )
        .setKinematics(DIFFERENTIAL_DRIVE_KINEMATICS)
        .addConstraint(RAMSETE_AUTO_VOLTAGE_CONSTRAINT);
    

    

    //Drivetrain
    public static final double DRIVETRAIN_MANUAL_SPEED_MODIFIER = 0.5;
    public static final boolean DRIVETRAIN_INVERT_FORWARD = false;
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MIN = 1;//CAN ID
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MAX = 2;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MIN = 4;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MAX = 5;//CAN ID
    
    public static final double DRIVETRAIN_TRACKING_KP = 0;
    public static final double DRIVETRAIN_TRACKING_KD = 0;



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
        new ManipulatorMoverSegment(2.1/10, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(10, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.right, -90, 90, new Motor[]{new CANSparkMax_Motor(11, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(12, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.right, -90, 90, new Motor[]{new CANSparkMax_Motor(13, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(14, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.right, -90, 90, new Motor[]{new CANSparkMax_Motor(15, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(16, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.right, -90, 90, new Motor[]{new CANSparkMax_Motor(17, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.forward, -90, 90, new Motor[]{new CANSparkMax_Motor(18, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.1/10, Vector3.right, -90, 90, new Motor[]{new CANSparkMax_Motor(19, CANSparkMaxLowLevel.MotorType.kBrushless)}, Motor.ControlMode.Percent),
        /*
        new ManipulatorMoverSegment(2.0/3, Vector3.forward, -90, 90, new Motor[]{new PWM_Motor(10, 0, 1, 360)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.0/3, Vector3.right, -90, 90, new Motor[]{new PWM_Motor(11, 2, 3, 360)}, Motor.ControlMode.Percent),
        new ManipulatorMoverSegment(2.0/3, Vector3.forward, -90, 90, new Motor[]{new PWM_Motor(12, 4, 5, 360)}, Motor.ControlMode.Percent),
        */
    };
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Utility.Motors.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Controls
    public static final Joystick mainController = new Joystick(0);

    public enum MotorType {
        PWM, CANSparkMax, TalonSRX
    };

    // Robot info (in meters)
    // the Pathweaver will want some of this information
    public static final double ROBOT_TRACKWIDTH = 0;//Pathweaver: wheel base
    public static final double ROBOT_WHEEL_DIAMETER = 0;
    public static final double ROBOT_WHEEL_RADIUS = 0;
    public static final double ROBOT_MAX_SPEED = 0;// set to something below the free-floating wheel speed
    public static final double ROBOT_MAX_ACCELERATION = 0;// not very important due to voltage limiting (but Pathweaver might not be using the voltage limiting)

    // Robot Characterization 
    // info on how to find this data: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
    public static final double KS_VOLTS = 0;// ks
    public static final double KV_VOLT_SECONDS_PER_METER = 0;// kv
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0;// ka
    public static final double KP_DRIVE_VEL = 0;// kp
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
    

    //Simulation control
    public static final boolean IS_SIMULATION_RUNNING = false;//when simulation is on all CAN motors should get a PWM counterpart with the same ID

    //Drivetrain
    public static final boolean DRIVETRAIN_INVERT_FORWARD = false;
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MIN = 1;//CAN ID
    public static final int DRIVETRAIN_LEFT_MOTOR_IDS_MAX = 2;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MIN = 4;//CAN ID
    public static final int DRIVETRAIN_RIGHT_MOTOR_IDS_MAX = 5;//CAN ID
    public static final boolean DRIVETRAIN_GYRO_REVERSED = false;
    public static final double DRIVETRAIN_TRACKING_KP = 0;
    public static final double DRIVETRAIN_TRACKING_KD = 0;


}

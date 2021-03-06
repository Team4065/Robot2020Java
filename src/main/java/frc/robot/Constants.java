/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * This is where the robot and the controller inputs are configured CAN IDs must
 * not repeat or else it will result in unexpected behaviors
 */
public class Constants {
    public static final boolean IS_GYRO_REVERSED = true;
    public static final boolean IS_SPY_ENABLED = true;

    //Shooter
    public static final int SHOOTER_LEFT_MASTER_ID = 0;
    public static final int SHOOTER_RIGHT_SLAVE_ID = 0;
    public static final double SHOOTER_KS = 0;
    public static final double SHOOTER_KV = 0;
    public static final double SHOOTER_KA = 0;
    public static final int SPIN_UP_RPM = 0;


    //Feeder 
    public static final int FEEDER_MOTOR_ID = 0;
    public static final int SERIALIZER_MOTOR_ID = 0;

    //Kicker
    public static final int KICKER_MOTOR_ID = 0;

    //Ramsete
    public static final double ROBOT_TRACKWIDTH = Double.NaN;
    public static final double RAMSETE_B = Double.NaN;
    public static final double RAMSETE_ZETA =  Double.NaN;
    public static final double KS_VOLTS = Double.NaN;
    public static final double KV_VOLT_SECONDS_PER_METER = Double.NaN;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = Double.NaN;
    public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(ROBOT_TRACKWIDTH);
    public static final double KP_DRIVE_VEL = Double.NaN;

    //Drivetrain
    public static final int DRIVETRAIN_FRONT_LEFT_ID = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_ID = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ID = 0;
    public static final int DRIVETRAIN_BACK_RIGHT_ID = 0;

    //Intake
    public static final int INTAKE_MOTOR_ID = 0;
    public static final int INTAKE_DEPLOY_ID = 0;
    public static final int INTAKE_RETRACT_ID = 0;
}

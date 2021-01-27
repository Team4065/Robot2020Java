// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public enum MotorType {
        PWM, CANSparkMax, TalonSRX
    };

    //Shooter
    public static final int SHOOTER_LEFT_MASTER_ID = 1;
    public static final int SHOOTER_RIGHT_SLAVE_ID = 1;
    public static final int SHOOTER_KP = 0;
    public static final int SHOOTER_KI = 0;
    public static final int SHOOTER_KFF = 0;
    public static final int SHOOTER_KD = 0;
    public static final int SPIN_UP_RPM = 0;


    //Feeder 
    public static final int FEEDER_MOTOR_ID = 1;
    public static final int SERIALIZER_MOTOR_ID = 1;

    //Kicker
    public static final int KICKER_MOTOR_ID = 1;

    //Controls
    public static final Joystick mainController = new Joystick(0);

    //Sensors
    public static final boolean GYRO_REVERSED = false;

    //Program info
    public static final double DELTA_TIME = 0.02;//The delay between updates in seconds (20ms)
    public static final boolean IS_SIMULATION_RUNNING = false;//when simulation is on all CAN motors should get a PWM counterpart with the same ID
}

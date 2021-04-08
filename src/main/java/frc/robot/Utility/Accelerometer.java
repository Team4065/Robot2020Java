// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class Accelerometer {
    private static AHRS accelerometer = new AHRS(SPI.Port.kMXP);

    public static double getRawXAcceleration(){
        return accelerometer.getRawAccelX();
    }

    public static double getRawYAcceleration(){
        return accelerometer.getRawAccelY();
    }

    public static double getRawZcceleration(){
        return accelerometer.getRawAccelZ();
    }
}
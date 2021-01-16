/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class Gyro {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope with alterations from Constants settings.
     * @return
     */
    public static double getAngle(){
        return gyro.getAngle() * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope without alterations from Constants settings.
     * @return
     */
    public static double getRawAngle(){
        return gyro.getAngle();
    }

    /**
     * Returns the heading of the gyroscope based on a compass.
     * @return
     */
    public static double getCompassHeading(){
        return gyro.getCompassHeading();
    }

    /**
     * Returns the heading of the gyroscope based on a compass and accelerometers
     * Reduces drift but less accurate than getAngle.
     * Faster than getCompassHeading.
     * @return
     */
    public static double getFusedHeading(){
        return gyro.getFusedHeading();
    }
}

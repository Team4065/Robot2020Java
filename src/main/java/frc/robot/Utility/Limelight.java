/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Add your docs here.
 */
public class Limelight {
    static public final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    //getters

    static boolean hasTarget(){
        return (limelight.getEntry("tv").getDouble(Double.NaN) > 0) ? true : false;
    }

    static double getHorizontalOffset(){
        return limelight.getEntry("tx").getDouble(Double.NaN);
    }

    static double getVerticalOffset(){
        return limelight.getEntry("ty").getDouble(Double.NaN);
    }

    static double getArea(){
        return limelight.getEntry("ta").getDouble(Double.NaN);
    }

    static double getRotation(){
        return limelight.getEntry("ts").getDouble(Double.NaN);
    }

    static double getShortLength(){
        return limelight.getEntry("tshort").getDouble(Double.NaN);
    }

    static double getLongLength(){
        return limelight.getEntry("tlong").getDouble(Double.NaN);
    }

    static double getHorizontalLength(){
        return limelight.getEntry("thor").getDouble(Double.NaN);
    }

    static double getVerticalLength(){
        return limelight.getEntry("tvert").getDouble(Double.NaN);
    }



    //setters

    /**
     * 
     * @param mode 0: pipeline setting, 1: force off, 2: force blink, 3: force on
     */
    static void setLEDMode(int mode){
        limelight.getEntry("ledMode").setNumber(mode);
    }

    static void enableVisionProcessing(){
        limelight.getEntry("camMode").setNumber(0.0);
    }

    static void disableVisionProcessing(){
        limelight.getEntry("camMode").setNumber(1.0);
    }

    /**
     * 
     * @param pipeline Selects which pipeline to use. Ranges from 0-9.
     */
    static void setPipeline(int pipeline){
        limelight.getEntry("pipeline").setNumber(pipeline);
    }
}

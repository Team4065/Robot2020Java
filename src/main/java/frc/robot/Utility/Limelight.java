// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Add your docs here.
 */
public class Limelight {
    static public final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    //getters

    public static boolean hasTarget(){
        return (limelight.getEntry("tv").getDouble(Double.NaN) > 0) ? true : false;
    }

    public static double getHorizontalOffset(){
        return limelight.getEntry("tx").getDouble(Double.NaN);
    }

    public static double getVerticalOffset(){
        return limelight.getEntry("ty").getDouble(Double.NaN);
    }

    public static double getArea(){
        return limelight.getEntry("ta").getDouble(Double.NaN);
    }

    public static double getRotation(){
        return limelight.getEntry("ts").getDouble(Double.NaN);
    }

    public static double getShortLength(){
        return limelight.getEntry("tshort").getDouble(Double.NaN);
    }

    public static double getLongLength(){
        return limelight.getEntry("tlong").getDouble(Double.NaN);
    }

    public static double getHorizontalLength(){
        return limelight.getEntry("thor").getDouble(Double.NaN);
    }

    public static double getVerticalLength(){
        return limelight.getEntry("tvert").getDouble(Double.NaN);
    }



    //setters

    /**
     * 
     * @param mode 0: pipeline setting, 1: force off, 2: force blink, 3: force on
     */
    public static void setLEDMode(int mode){
        limelight.getEntry("ledMode").setNumber(mode);
    }

    public static void enableVisionProcessing(){
        limelight.getEntry("camMode").setNumber(0.0);
    }

    public static void disableVisionProcessing(){
        limelight.getEntry("camMode").setNumber(1.0);
    }

    /**
     * 
     * @param pipeline Selects which pipeline to use. Ranges from 0-9.
     */
    public static void setPipeline(int pipeline){
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getPipeline(){
        return (int)limelight.getEntry("pipeline").getDouble(Double.NaN);
    }
}

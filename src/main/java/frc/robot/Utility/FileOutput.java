/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.subsystems.manipulator_mover.ManipulatorMoverSegment;

import java.io.*;

/**
 * Add your docs here.
 */
public class FileOutput {
    static public void printManipulatorMoverState(String filePath, ManipulatorMover segmentSystem){
        try (FileWriter outFile = new FileWriter(filePath, true);
            BufferedWriter bWriter = new BufferedWriter(outFile);
            PrintWriter out = new PrintWriter(bWriter)) {

            ManipulatorMoverSegment[] segments = segmentSystem.getSegments();

            out.println();
            out.println(segmentSystem.getAnchor().toCSV());
            for(ManipulatorMoverSegment segment : segments){
                out.println(segment.getRobotspaceEnd().toCSV());
            }        

        } catch (IOException e) {
        e.printStackTrace();
        }
        //System.out.println("printing");
    }

    static public void clearFile(String filePath){
        try (FileWriter outFile = new FileWriter(filePath, false);
            BufferedWriter bWriter = new BufferedWriter(outFile);
            PrintWriter out = new PrintWriter(bWriter)) {

                //out.println();

        } catch (IOException e) {
        e.printStackTrace();
        }
    }
}

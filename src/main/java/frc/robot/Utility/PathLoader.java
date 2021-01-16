/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/**
 * A loader for path weaver trajectories
 */
public class PathLoader {

    private Trajectory trajectory;

    /**
     * Loads the data from a path weaver file.
     * @param pathDirectory The directory of the file with the desired Ramsete path.
     */
    public PathLoader(String pathDirectory){
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathDirectory);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathDirectory, ex.getStackTrace());
          }
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }
}

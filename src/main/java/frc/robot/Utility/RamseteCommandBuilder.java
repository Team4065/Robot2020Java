/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.differential_drivetrain.Drivetrain;

/**
 * Add your docs here.
 */
public class RamseteCommandBuilder {

    private RamseteCommand command;
    /**
     * 
     * @param drivetrain
     * @param path
     */
    public RamseteCommandBuilder(Drivetrain drivetrain, PathLoader path){
        command = new RamseteCommand(
            path.getTrajectory(), 
            drivetrain::getPose,
            new RamseteController(RobotMap.RAMSETE_B, RobotMap.RAMSETE_ZETA),
            new SimpleMotorFeedforward(RobotMap.KS_VOLTS, RobotMap.KV_VOLT_SECONDS_PER_METER, RobotMap.KA_VOLT_SECONDS_SQUARED_PER_METER),
            RobotMap.DIFFERENTIAL_DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(RobotMap.KP_DRIVE_VEL, 0, 0),
            new PIDController(RobotMap.KP_DRIVE_VEL, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);
    }

    public RamseteCommand getCommand(){
        return command;
    }
}

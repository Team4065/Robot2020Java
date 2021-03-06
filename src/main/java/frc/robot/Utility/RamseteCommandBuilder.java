// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain;

/**
 * Makes the Ramsete command that is to be run.
 */
public class RamseteCommandBuilder {

    private RamseteCommand command;
    /**
     * Makes the Ramsete command that is to be run.
     * @param drivetrain The drivetrain subsystem.
     * @param path The PathLoader with the desired Ramsete path.
     */
    public RamseteCommandBuilder(DifferentialDrivetrain drivetrain, PathLoader path){
        /*
        command = new RamseteCommand(
            path.getTrajectory(), 
            drivetrain::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            //new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
            drivetrain::tankDriveMeterPerSecond,
            drivetrain);
        */
        command = new RamseteCommand(
            path.getTrajectory(), 
            drivetrain::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
            drivetrain::setTankDriveVolts,
            drivetrain);
            
        
    }

    public RamseteCommand getCommand(){
        return command;
    }
}
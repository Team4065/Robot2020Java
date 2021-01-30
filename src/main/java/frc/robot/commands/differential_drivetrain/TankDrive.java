/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.differential_drivetrain;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.differential_drivetrain.*;


public class TankDrive extends CommandBase {
  /**
   * Creates a new TankDrive.
   */
  Drivetrain drivetrain;

  public TankDrive(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drivetrain);
    drivetrain = _drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setControlMode(Drivetrain.ControlMode.PERCENT);
    //drivetrain.setControlMode(Drivetrain.ControlMode.VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setLeftTarget(Constants.DRIVETRAIN_MANUAL_SPEED_MODIFIER * -Constants.mainController.getRawAxis(1));
    drivetrain.setRightTarget(Constants.DRIVETRAIN_MANUAL_SPEED_MODIFIER * -Constants.mainController.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

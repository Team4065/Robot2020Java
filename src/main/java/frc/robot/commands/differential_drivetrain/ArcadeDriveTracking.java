/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.differential_drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.differential_drivetrain.Drivetrain;
import frc.robot.Utility.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ArcadeDriveTracking extends CommandBase {
  
  Drivetrain drivetrain;
  double pastError = 0;
  double deltaError = 0;

  public ArcadeDriveTracking(Drivetrain _drivetrain) {
    addRequirements(_drivetrain);
    drivetrain = _drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pastError = 0;
    this.deltaError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = Limelight.getHorizontalOffset();
    deltaError = error - pastError;

    double output = (Constants.DRIVETRAIN_TRACKING_KP * error) + (Constants.DRIVETRAIN_TRACKING_KD * deltaError);
    double speed = Constants.mainController.getY(Hand.kLeft);
    double rotation = Constants.mainController.getX(Hand.kRight);
    drivetrain.setLeftTarget(output + speed - rotation);
    drivetrain.setRightTarget(-output + speed + rotation);

    pastError = error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftTarget(0);//saftey
    drivetrain.setRightTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

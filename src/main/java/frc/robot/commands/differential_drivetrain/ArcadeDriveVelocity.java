// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.differential_drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.differential_drivetrain.Drivetrain;

public class ArcadeDriveVelocity extends CommandBase {
  Drivetrain m_drivetrain;

  /** Creates a new ArcadeDriveVelocity. */
  public ArcadeDriveVelocity(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(Drivetrain.ControlMode.VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double speed = -Constants.mainController.getY(Hand.kLeft);
    double rotation = -Constants.mainController.getRawAxis(4);
    if(Math.abs(speed) < 0.05)
      speed = 0;
    if(Math.abs(rotation) < 0.05)
      rotation = 0;
    
    speed *= Constants.DRIVETRAIN_MANUAL_SPEED_MODIFIER;
    rotation *= Constants.DRIVETRAIN_MANUAL_SPEED_MODIFIER;

    double left = speed - rotation;
    double right = speed + rotation;
    
    m_drivetrain.setLeftTarget(left);
    m_drivetrain.setRightTarget(right);
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

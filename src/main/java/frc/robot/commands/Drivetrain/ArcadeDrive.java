// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

public class ArcadeDrive extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  Joystick m_controller;
  double m_maxSpeed, m_maxRotation;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DifferentialDrivetrain drivetrain, Joystick controller, double maxSpeed, double maxRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_controller = controller;
    m_maxSpeed = maxSpeed;
    m_maxRotation = maxRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_controller.getRawAxis(1);
    double rotation = m_controller.getRawAxis(4);

    if(Math.abs(speed) < 0.05)
      speed = 0;
    if(Math.abs(rotation) < 0.05)
      rotation = 0;
    
    speed *= m_maxSpeed;
    rotation *= m_maxRotation;

    m_drivetrain.setArcadeDrive(-speed, -rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

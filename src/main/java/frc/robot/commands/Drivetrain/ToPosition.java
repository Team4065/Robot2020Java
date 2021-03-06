// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

public class ToPosition extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  PIDController m_PIDLeft = new PIDController(5, 0, 0);
  PIDController m_PIDRight = new PIDController(5, 0, 0);
  /** Creates a new ToPosition. */
  public ToPosition(DifferentialDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_PIDLeft.calculate(m_drivetrain.getLeftPosition(), 1);
    double right = m_PIDRight.calculate(m_drivetrain.getRightPosition(), 1);
    m_drivetrain.setLeftTarget(left);
    m_drivetrain.setRightTarget(right);
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
    return m_PIDLeft.atSetpoint() && m_PIDRight.atSetpoint();
  }
}

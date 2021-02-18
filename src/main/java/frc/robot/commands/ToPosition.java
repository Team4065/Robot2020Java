// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain2;
import frc.robot.subsystems.DifferentialDrivetrain2.ControlMode;

public class ToPosition extends CommandBase {
  DifferentialDrivetrain2 m_drivetrain;
  PIDController m_PID = new PIDController(0.01, 0, 0);
  /** Creates a new ToPosition. */
  public ToPosition(DifferentialDrivetrain2 drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_PID.calculate(m_drivetrain.getLeftPosition(), 1);
    m_drivetrain.setLeftTarget(left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Gyro;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

public class ToRotation extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  PIDController m_PID = new PIDController(-5, 0, -0.6);
  double m_target = 0;
  /** Creates a new ToRotation. */
  public ToRotation(DifferentialDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_PID.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
    m_target = Gyro.getAngle() + 90;
    
    //Gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_PID.calculate(Gyro.getAngle(), m_target);
    if(Math.abs(rotation) >= 270)
      rotation = Math.signum(rotation) * 270;
    m_drivetrain.setArcadeDrive(0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_PID.atSetpoint();
  }
}

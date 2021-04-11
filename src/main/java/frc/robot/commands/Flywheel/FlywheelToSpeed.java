// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class FlywheelToSpeed extends CommandBase {
  Flywheel m_flywheel;
  double m_speed;
  /** Creates a new FlywheelToSpeed. */
  public FlywheelToSpeed(Flywheel flywheel,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setVelocity(m_speed);
    //System.out.println(m_flywheel.getVelocity());
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

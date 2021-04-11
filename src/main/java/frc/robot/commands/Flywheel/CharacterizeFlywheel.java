// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class CharacterizeFlywheel extends CommandBase {
  Flywheel m_flywheel;
  double m_voltage;
  double m_pastVelocity;
  /** Creates a new CharacterizeFlywheel. */
  public CharacterizeFlywheel(Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pastVelocity = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setVoltage(m_voltage);

    System.out.printf("%f", m_voltage);
    System.out.print(",");
    System.out.printf("%f", m_flywheel.getVelocity());
    System.out.print(",");
    System.out.printf("%f", (m_flywheel.getVelocity() - m_pastVelocity) / (20.0 / 1000.0) /* deltatime */);
    System.out.println();
    m_voltage += 0.01;
    m_pastVelocity = m_flywheel.getVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_voltage >= 12;
  }
}

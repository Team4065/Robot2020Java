/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;

public class KickerOn extends CommandBase {
  Kicker m_kicker;
  /**
   * Creates a new KickerOn.
   */
  public KickerOn(Kicker kicker) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_kicker = kicker;
    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_kicker.on();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_kicker.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

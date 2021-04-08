// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class LiftRetract extends CommandBase {
  private Lift m_lift;
  /** Creates a new LiftRetract. */
  public LiftRetract(Lift lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
    m_lift = lift;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //This prevents the bar on the bottom of the lift from crashing into the robot.
    if(m_lift.getHeight() <= 0.1){
      m_lift.retract();
    }
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

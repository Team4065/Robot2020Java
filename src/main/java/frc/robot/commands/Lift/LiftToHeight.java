// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class LiftToHeight extends CommandBase {
  Lift m_lift;
  Double m_target;
  /** Creates a new LiftToHeight. */
  public LiftToHeight(Lift lift, double percentHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
    m_lift = lift;
    if(percentHeight > 0.9){
      m_target = 0.9;
    }else{
      if(percentHeight < 0){
        m_target = 0.0;
      }else{
        m_target = percentHeight;
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_lift.getHeight() > m_target){
      m_lift.moveDown();
    }
    if(m_lift.getHeight() < m_target){
      m_lift.moveUp();
    }
    //System.out.println(m_lift.getHeight());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_lift.getHeight() - m_target) < 0.01;
  }
}

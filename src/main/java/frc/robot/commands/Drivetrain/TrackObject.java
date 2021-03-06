// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Limelight;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

public class TrackObject extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  PIDController m_PID = new PIDController(-8, 0, -0.88399);
  public enum WhatToTrack {
    Ball,
    Tape
  };

  WhatToTrack m_toTrack;

  /** Creates a new TrackObject. */
  public TrackObject(DifferentialDrivetrain drivetrain, WhatToTrack whatToTrack) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_PID.setTolerance(1);
    m_toTrack = whatToTrack;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);

    switch(m_toTrack){
      case Ball:
        Limelight.setPipeline(1);
        break;
      case Tape:
        Limelight.setPipeline(2);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_PID.calculate(Limelight.getHorizontalOffset(), 0);

    if(Math.abs(rotation) >= 270)
      rotation = Math.signum(rotation) * 270;
    //if(Math.abs(rotation) < 10)
    //  rotation = Math.signum(rotation) * 10;
    
    m_drivetrain.setArcadeDrive(0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setArcadeDrive(0, 0);

    Limelight.setPipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_PID.atSetpoint() && Limelight.hasTarget();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Gyro;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

public class CharacterizeRotation extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  double targetVelocity = 0;
  double pastRate = 0;

  /** Creates a new CharacterizeRotation. */
  public CharacterizeRotation(DifferentialDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
    Gyro.reset();
    targetVelocity = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setLeftTarget(targetVelocity);
    m_drivetrain.setRightTarget(-targetVelocity);
    targetVelocity += 0.001;
    System.out.print(targetVelocity);
    System.out.print(",");
    System.out.print(Gyro.getRate());
    System.out.print(",");
    System.out.println((pastRate - Gyro.getRate()) / (20.0 / 1000.0));
    
    
    pastRate = Gyro.getRate();
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

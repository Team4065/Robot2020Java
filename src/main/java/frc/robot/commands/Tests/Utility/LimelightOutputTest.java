/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tests.Utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Limelight;
import frc.robot.Utility.Spy;

public class LimelightOutputTest extends CommandBase {
  /**
   * Creates a new LimelightOutputTest.
   */
  public LimelightOutputTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Spy.getSpy());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Has target", Limelight.hasTarget());
    // SmartDashboard.putNumber("Horizontal offset", Limelight.getHorizontalOffset());
    // SmartDashboard.putNumber("Vertical offset", Limelight.getVerticalOffset());
    // SmartDashboard.putNumber("Area", Limelight.getArea());
    // SmartDashboard.putNumber("Rotation", Limelight.getRotation());
    // SmartDashboard.putNumber("Length of shortest fitted side", Limelight.getShortLength());
    // SmartDashboard.putNumber("Length of longest fitted side", Limelight.getLongLength());
    // SmartDashboard.putNumber("Horizontal side length", Limelight.getHorizontalLength());
    // SmartDashboard.putNumber("Vertical side length", Limelight.getVerticalLength());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.delete("Has target");
    // SmartDashboard.delete("Horizontal offset");
    // SmartDashboard.delete("Vertical offset");
    // SmartDashboard.delete("Area");
    // SmartDashboard.delete("Rotation");
    // SmartDashboard.delete("Length of shortest fitted side");
    // SmartDashboard.delete("Length of longest fitted side");
    // SmartDashboard.delete("Horizontal side length");
    // SmartDashboard.delete("Vertical side length");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tests.differential_drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.differential_drivetrain.Drivetrain;
import frc.robot.Utility.Spy;

public class DisplayDrivetrainOutputs extends CommandBase {

  Drivetrain drivetrain;

  public DisplayDrivetrainOutputs(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = _drivetrain;
    addRequirements(Spy.getSpy());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumberArray("Left motors", drivetrain.getLeftOutputs());
    //SmartDashboard.putNumberArray("Right motors", drivetrain.getRightOutputs());
    //SmartDashboard.putNumber("hi", 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.delete("Left motors");
    //SmartDashboard.delete("Right motors");
    //SmartDashboard.delete("hi");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

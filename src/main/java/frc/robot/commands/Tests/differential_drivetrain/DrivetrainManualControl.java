/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tests.differential_drivetrain;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.differential_drivetrain.Drivetrain;

public class DrivetrainManualControl extends CommandBase {

  Drivetrain drivetrain;

  public DrivetrainManualControl(Drivetrain _drivetrain) {
    addRequirements(_drivetrain);
    drivetrain = _drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShuffleboardLayout layout = Shuffleboard.getTab("Commands")
      .getLayout("Drivetrain Output", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "HIDDEN"));
    
    layout.add("Left output", 0);
    layout.add("Right output", 0);
    //SmartDashboard.putNumber("Left output", 0);
    //SmartDashboard.putNumber("Right output", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.setLeftTarget(SmartDashboard.getNumber("Left output", 0));
    //drivetrain.setRightTarget(SmartDashboard.getNumber("Right output", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

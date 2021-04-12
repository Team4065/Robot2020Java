// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.ConveyorIn;
import frc.robot.commands.Feeder.FeedAmmo;
import frc.robot.commands.Feeder.PrepareAmmo;
import frc.robot.commands.Flywheel.FlywheelMaintainSpeed;
import frc.robot.commands.Flywheel.FlywheelToSpeed;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Flywheel flywheel, Feeder feeder, Conveyor conveyor, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new FlywheelToSpeed(flywheel, 80),
      //new FlywheelMaintainSpeed(flywheel)
      new ParallelRaceGroup(new FlywheelToSpeed(flywheel, speed), new PrepareAmmo(feeder), new ConveyorIn(conveyor)),
      new ParallelRaceGroup(new FlywheelMaintainSpeed(flywheel), new FeedAmmo(feeder), new ConveyorIn(conveyor))
    );
  }
}

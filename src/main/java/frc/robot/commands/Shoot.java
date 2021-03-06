/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, Kicker kicker, Feeder feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(
      new SpinUpFlyWheel(shooter),
      new ParallelCommandGroup(new MaintainFlyWheel(shooter), new KickerOn(kicker), new FeederOn(feeder))

    );
  }
}

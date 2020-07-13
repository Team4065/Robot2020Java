/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator_mover;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.ExtraMath.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Path2 extends SequentialCommandGroup {
  /**
   * Creates a new Path2.
   */
  public Path2(ManipulatorMover manipulatorMover) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new GoToState(manipulatorMover, new Vector3[]{new Vector3(0, 0.5, 0), new Vector3(0, 1, 0), new Vector3(0, 1.5, 0), new Vector3(0, 2, 0)}, 1),
      new GoToState(manipulatorMover, new Vector3[]{new Vector3(0, 0.5, 0), new Vector3(0, 0.7, 0.4), new Vector3(0.4, 0.8, 0.6), new Vector3(0.7, 1.25, 0.6)}, 1),
      new GoToState(manipulatorMover, new Vector3[]{new Vector3(0.3, 0.4, 0), new Vector3(0.55, 0.8, 0), new Vector3(0.6, 1.3, 0), new Vector3(0.6, 1.8, 0)}, 1),
      new GoToState(manipulatorMover, new Vector3[]{new Vector3(0, 0.5, 0), new Vector3(0, 1, 0), new Vector3(0, 1.5, 0), new Vector3(0, 2, 0)}, 1)
    );
  }
}

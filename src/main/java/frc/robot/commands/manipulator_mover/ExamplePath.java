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
public class ExamplePath extends SequentialCommandGroup {

  public ExamplePath(ManipulatorMover manipulatorMover) {

    super(//            manipulator mover  location                speed
      new GoToPosition( manipulatorMover,  new Vector3(1, 1, 1),   1),
      new GoToPosition( manipulatorMover,  new Vector3(-1, 1, 1),  2),
      new GoToPosition( manipulatorMover,  new Vector3(0, 2, 0),   1),
      new GoToPosition( manipulatorMover,  new Vector3(0, 1, -1),  1)
      );
  }
}

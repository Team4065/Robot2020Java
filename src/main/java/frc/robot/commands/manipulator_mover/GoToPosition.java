/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator_mover;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.Constants;
import frc.robot.Utility.Vector3;

/**
 * Sets the target position of the last segement
 */
public class GoToPosition extends CommandBase {

  private ManipulatorMover manipulatorMover;
  private Vector3 target;
  private double speed;

  /**
   * Moves the target position at a set rate
   * @param _manipulatorMover the manipulator mover to be controlled
   * @param _target the desired endpoint
   * @param _speed the speed in units per second
   */
  public GoToPosition(ManipulatorMover _manipulatorMover, Vector3 _target, double _speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_manipulatorMover);
    manipulatorMover = _manipulatorMover;
    target = _target;
    speed = _speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //disables all subtargets
    for(int i = 0; i < manipulatorMover.getSegments().length - 1; ++i){
      manipulatorMover.disableSubTarget(i);
    }
    //enables inverse kinematics
    manipulatorMover.enableInverseKinematics();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //get target prevents the system from stopping due to temporary reverse motion
    Vector3 tempTarget = Vector3.moveTowards(manipulatorMover.getTarget(), target, speed * Constants.DELTA_TIME);

    /*
    double timeJump = 1;
    Vector3 tempTarget = Vector3.slerp(manipulatorMover.getTarget(), target, timeJump);
    while(tempTarget.sub(manipulatorMover.getTarget()).magnitude() >  speed * Constants.DELTA_TIME){
      timeJump *= 0.75;
      tempTarget = Vector3.slerp(manipulatorMover.getTarget(), target, timeJump);
    }
    */
    manipulatorMover.setTarget(tempTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //disables inverse kinematics to prevent unwanted moving
    manipulatorMover.disableInverseKinematics();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulatorMover.getMeasuredFinalEndpoint().sub(target).magnitude() < Constants.MANIPULATOR_MOVER_ACCURACY_TOLERANCE;
  }
}

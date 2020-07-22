/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator_mover;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.ExtraMath.*;
import frc.robot.RobotMap;

public class GoToState extends CommandBase {
  private ManipulatorMover manipulatorMover;
  private Vector3[] targets;
  private double speed;

  /**
   * Moves the target position at a set rate
   * @param _manipulatorMover the manipulator mover to be controlled
   * @param _targets the desired endpoint
   * @param _speed the speed in units per second
   */
  public GoToState(ManipulatorMover _manipulatorMover, Vector3[] _targets, double _speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_manipulatorMover);
    manipulatorMover = _manipulatorMover;
    targets = _targets;
    speed = _speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //enables all subtargets
    for(int i = 0; i < manipulatorMover.getSegments().length - 1; ++i){
      manipulatorMover.enableSubTarget(i);
    }
    //enables inverse kinematics
    manipulatorMover.enableInverseKinematics();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //changes all the subtargets and the target
    Vector3[] currentState = manipulatorMover.getMeasuredEndpoints();
    Vector3[] tempTargets = new Vector3[currentState.length];
    for(int i = 0; i < tempTargets.length; ++i){
      tempTargets[i] = Vector3.moveTowards(currentState[i], targets[i], speed * RobotMap.DELTA_TIME);
     
      if(i < tempTargets.length - 1){
        manipulatorMover.setSubTarget(i, tempTargets[i]);//for all but the last segment
      }else{
        manipulatorMover.setTarget(tempTargets[i]);
      }
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //disables all subtargets
    for(int i = 0; i < manipulatorMover.getSegments().length - 1; ++i){
      manipulatorMover.disableSubTarget(i);
    }
    //disables inverse kinematics to prevent unwanted moving
    manipulatorMover.disableInverseKinematics();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Vector3[] currentState = manipulatorMover.getMeasuredEndpoints();
    boolean isDone = true;
    for(int i = 0; i < currentState.length; ++i){
      if(currentState[i].sub(targets[i]).magnitude() > RobotMap.MANIPULATOR_MOVER_ACCURACY_TOLERANCE){
        isDone = false;
        break;
      }
    }
    return isDone;
  }
}

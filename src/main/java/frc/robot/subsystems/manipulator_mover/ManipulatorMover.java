/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.manipulator_mover;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath.*;
import frc.robot.Utility.FileOutput;


public class ManipulatorMover extends SubsystemBase {
  /**
   * Creates a new ManipulatorMover.
   */

  ManipulatorMoverSegment[] segments;
  private Vector3 target;//the desired end point of the segment system (robot space)
  private Vector3 anchor = Vector3.zero;//the starting point of the segment system (robot space)
  private boolean isInverseKinematicsEnabled = true/*false*/;

  private Vector3[] subTargets;//the desired end point of each segment (robot space) ([0] = base segment. [last] = second to last segment)
  private boolean areSubTargetsEnabled = true;

  public ManipulatorMover(ManipulatorMoverSegment[] _segments) {
    segments = _segments;
    for(int i = 0; i < segments.length - 1; ++i){
      segments[i].setChildSegment(segments[i + 1]);
    }

    segments[0].forwardKinematics(anchor);//calculates robotspace variables which prevents null pointer errors
    
    subTargets = new Vector3[segments.length - 1];

    target = segments[segments.length - 1].getRobotspaceEnd();
  }

  @Override
  public void periodic() {
    
    if(isInverseKinematicsEnabled){
      inverseKinematics();

      //for testing
      FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
    }
  }

  public void gotoInitialRobotState(){}

  public void gotoInitialTeleopState(){}

  public void gotoInitialAutonomusState(){}

  public void setTarget(Vector3 _target){
    target = _target;
  }

  public void setTarget(double x, double y, double z){
    target = new Vector3(x, y, z);
  }

  public Vector3 getTarget(){
    return target;
  }

  public void setAnchor(Vector3 _anchor){
    anchor = _anchor;
  }

  public void setAnchor(double x, double y, double z){
    anchor = new Vector3(x, y, z);
  }

  public Vector3 getAnchor(){
    return anchor;
  }

  public ManipulatorMoverSegment[] getSegments(){
    return segments;
  }

  private void inverseKinematics(){
    segments[segments.length - 1].inverseKinematics(target, anchor);
    for(ManipulatorMoverSegment segment : segments){
      segment.realign();
    }
    segments[0].forwardKinematics(anchor);
  }
  
  private void subInverseKinematics(){
    for(int i = 0; i < subTargets.length; ++i){
      segments[i].inverseKinematics(subTargets[i], anchor);
      for(ManipulatorMoverSegment segment : segments){
        segment.realign();
      }
      segments[0].forwardKinematics(anchor);
    }
  }

  /**
   * incomplete
   * @return
   */
  public Vector3 getMeasuredEndpoint(){
    return target;
  }
  /*for testing*/
  public void updateKinematics(){
    segments[0].forwardKinematics(anchor);
    inverseKinematics();
    FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
  }


}

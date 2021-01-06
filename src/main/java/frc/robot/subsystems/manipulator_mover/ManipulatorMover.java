/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.manipulator_mover;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.ExtraMath.*;
import frc.robot.Utility.FileOutput;
import frc.robot.Utility.Motors.*;


public class ManipulatorMover extends SubsystemBase {
  /**
   * Creates a new ManipulatorMover.
   */

  ManipulatorMoverSegment[] segments;
  private Vector3 target;//the desired end point of the segment system (robot space)
  private Vector3 anchor = Vector3.zero;//the starting point of the segment system (robot space)
  private boolean isInverseKinematicsEnabled = false;
  private boolean areMotorsEnabled = false;

  private Vector3[] subTargets;//the desired end point of each segment (robot space) ([0] = base segment. [last] = second to last segment)
  private boolean[] enabledSubTargets;//the subtargets that are enabled


  public ManipulatorMover() {     
    segments = RobotMap.MANIPULATOR_MOVER_SEGMENTS;//gets the data for the segments
    
    //sets up segments so that they know how they relate to the other segments
    for(int i = 0; i < RobotMap.MANIPULATOR_MOVER_SEGMENTS.length - 1; ++i){
      segments[i].setChildSegment(segments[i + 1]);
    }

    segments[0].forwardKinematics(anchor);//calculates robotspace variables which prevents null pointer errors
    
    //setup for sub targets
    subTargets = new Vector3[RobotMap.MANIPULATOR_MOVER_SEGMENTS.length - 1];
    enabledSubTargets = new boolean[RobotMap.MANIPULATOR_MOVER_SEGMENTS.length - 1];
    for(int i = 0; i < subTargets.length; ++i){
      subTargets[i] = segments[i].getRobotspaceEnd();
    }
    for(boolean b : enabledSubTargets){
      b = false;
    }

    target = segments[RobotMap.MANIPULATOR_MOVER_SEGMENTS.length - 1].getRobotspaceEnd();//Sets the initial target to the natural end position of the segment system.
    segments[0].measuredForwardKinematics(anchor);
  }

  @Override
  public void periodic() {
    if(isInverseKinematicsEnabled){//only runs when needed
      inverseKinematics();//finds the angles that each segment needs to be at to reach the target
      subInverseKinematics();//finds the angles that each segment needs to be at to reach the subtargets

      

      //for testing
      //FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
    }

    if(areMotorsEnabled){
      //updates the motors on the segments
      for(ManipulatorMoverSegment segment : segments){
        segment.update();
      }
    }else{
      //disables the motors (sets the output to 0)
      for(ManipulatorMoverSegment segment : segments){
        segment.disabledUpdate();
      }
    }
  }

  public void enableInverseKinematics(){
    isInverseKinematicsEnabled = true;
  }

  public void disableInverseKinematics(){
    isInverseKinematicsEnabled = false;
  }

  public void enableMotors(){
    areMotorsEnabled = true;
  }

  public void disableMotors(){
    areMotorsEnabled = false;
  }

  /**
   * Sets the target position for the end of the segment system.
   * @param _target
   */
  public void setTarget(Vector3 _target){
    target = _target;
  }

  /**
   * Sets the target position for the end of the segment system.
   * @param x
   * @param y
   * @param z
   */
  public void setTarget(double x, double y, double z){
    target = new Vector3(x, y, z);
  }

  /**
   * 
   * @return The target position for the end of the segment system.
   */
  public Vector3 getTarget(){
    return target;
  }


  public void setSubTarget(int targetIndex, Vector3 _target){
    subTargets[targetIndex] = _target;
  }

  public Vector3 getSubTarget(int targetIndex){
    return subTargets[targetIndex];
  }

  public void enableSubTarget(int targetIndex){
    enabledSubTargets[targetIndex] = true;
  }

  public void disableSubTarget(int targetIndex){
    enabledSubTargets[targetIndex] = false;
  }


  /**
   * Sets the location of the start of the segment system.
   * @param _anchor
   */
  public void setAnchor(Vector3 _anchor){
    anchor = _anchor;
  }

  /**
   * Sets the location of the start of the segment system.
   * @param x
   * @param y
   * @param z
   */
  public void setAnchor(double x, double y, double z){
    anchor = new Vector3(x, y, z);
  }

  /**
   * 
   * @return The location of the start of the segment system in robotspace.
   */
  public Vector3 getAnchor(){
    return anchor;
  }

  public ManipulatorMoverSegment[] getSegments(){
    return segments;
  }

  /**
   * Applies the inverse kinematics algorithm to the segment system.
   */
  private void inverseKinematics(){
    segments[segments.length - 1].inverseKinematics(target, anchor);//sets the target of the last segment
    for(ManipulatorMoverSegment segment : segments){
      segment.realign();//constrains the segments to their axes of rotation
    }
    segments[0].forwardKinematics(anchor);//recalculates the positions of the segments
  }
  
  /**
   * Applies individual targets for each child segment.
   * Only applies targets to child segments that have been selected to have an individual target.
   */
  private void subInverseKinematics(){
    for(int i = subTargets.length - 1; i >= 0; --i){
      if(enabledSubTargets[i]){//only runs the activated targets
        //runs the inverse kinematics algorithm with the different targets
        segments[i].inverseKinematics(subTargets[i], anchor);
        for(ManipulatorMoverSegment segment : segments){
          segment.realign();
        }
        segments[0].forwardKinematics(anchor);

      }
    }
  }

  /**
   * 
   * @return The endpoint of the last segment in robotspace.
   */
  public Vector3 getMeasuredFinalEndpoint(){
    if(RobotMap.IS_SIMULATION_RUNNING){
      return segments[segments.length - 1].getRobotspaceEnd();//if simulation is running there are no encoders to measure from
    }else{
      segments[0].measuredForwardKinematics(anchor);//calulates the actual endpoints of each segment
      return segments[segments.length - 1].getMeasuredRobotspaceEnd();
    }
  }

  /**
   * 
   * @return The endpoints of every segment in robotspace.
   */
  public Vector3[] getMeasuredEndpoints(){
    Vector3[] endpoints = new Vector3[segments.length]; 
    if(RobotMap.IS_SIMULATION_RUNNING){
      for(int i = 0; i < segments.length; ++i){
        endpoints[i] = segments[i].getRobotspaceEnd();//if simulation is running there are no encoders to measure from
      }
    }else{
      segments[0].measuredForwardKinematics(anchor);//calulates the actual endpoints of each segment
      for(int i = 0; i < segments.length; ++i){
        endpoints[i] = segments[i].getMeasuredRobotspaceEnd();
      }
    }

    return endpoints;
  }

  public double[] getMeasuredAngles(){
    double[] angles = new double[segments.length]; 
    if(RobotMap.IS_SIMULATION_RUNNING){
      for(int i = 0; i < segments.length; ++i){
        angles[i] = segments[i].getAngle();//if simulation is running there are no encoders to measure from
      }
    }else{
      for(int i = 0; i < segments.length; ++i){
        angles[i] = segments[i].getMeasuredAngle();
      }
    }

    return angles;
  }


  /*for testing*/
  /*
  public void updateKinematics(){
    inverseKinematics();
    subInverseKinematics();
    FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
  }
  */
}

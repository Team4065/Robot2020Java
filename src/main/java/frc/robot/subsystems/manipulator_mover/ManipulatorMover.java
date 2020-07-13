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


public class ManipulatorMover extends SubsystemBase {
  /**
   * Creates a new ManipulatorMover.
   */

  ManipulatorMoverSegment[] segments;
  private Vector3 target;//the desired end point of the segment system (robot space)
  private Vector3 anchor = Vector3.zero;//the starting point of the segment system (robot space)
  private boolean isInverseKinematicsEnabled = true/*false*/;

  private Vector3[] subTargets;//the desired end point of each segment (robot space) ([0] = base segment. [last] = second to last segment)
  private boolean[] enabledSubTargets;//the subtargets that are enabled


  public ManipulatorMover() {
    switch(RobotMap.MANIPULATOR_MOVER_MOTOR_TYPE){
      case PWM:
        segments = new PWM_ManipulatorMoverSegment[RobotMap.MANIPULATOR_MOVER_MOTOR_IDS.length];
        for(int i = 0; i < segments.length; ++i){
          segments[i] = new PWM_ManipulatorMoverSegment(
            RobotMap.MANIPULATOR_MOVER_LENGTHS[i],
            RobotMap.MANIPULATOR_MOVER_AXES[i],
            RobotMap.MANIPULATOR_MOVER_MIN_ANGLES[i],
            RobotMap.MANIPULATOR_MOVER_MAX_ANGLES[i],
            RobotMap.MANIPULATOR_MOVER_MOTOR_IDS[i],
            RobotMap.MANIPULATOR_MOVER_MOTOR_INVERSIONS[i],
            RobotMap.MANIPULATOR_MOVER_ENCODER_CHANNELS[i]
            );
        }
        break;
      case CANSparkMax:
        segments = new PWM_ManipulatorMoverSegment[RobotMap.MANIPULATOR_MOVER_MOTOR_IDS.length];
        for(int i = 0; i < segments.length; ++i){

        }
        break;
      case TalonSRX:
        segments = new PWM_ManipulatorMoverSegment[RobotMap.MANIPULATOR_MOVER_MOTOR_IDS.length];
        for(int i = 0; i < segments.length; ++i){
          
        }
        break;
    }
      


    for(int i = 0; i < segments.length - 1; ++i){
      segments[i].setChildSegment(segments[i + 1]);
    }

    segments[0].forwardKinematics(anchor);//calculates robotspace variables which prevents null pointer errors
    
    //setup for sub targets
    subTargets = new Vector3[segments.length - 1];
    enabledSubTargets = new boolean[segments.length - 1];
    for(int i = 0; i < subTargets.length; ++i){
      subTargets[i] = segments[i].getRobotspaceEnd();
    }
    for(boolean b : enabledSubTargets){
      b = false;
    }

    target = segments[segments.length - 1].getRobotspaceEnd();
  }

  /*
  public ManipulatorMover(ManipulatorMoverSegment[] _segments) {
    segments = _segments;
    for(int i = 0; i < segments.length - 1; ++i){
      segments[i].setChildSegment(segments[i + 1]);
    }

    segments[0].forwardKinematics(anchor);//calculates robotspace variables which prevents null pointer errors
    
    //setup for sub targets
    subTargets = new Vector3[segments.length - 1];
    enabledSubTargets = new boolean[segments.length - 1];
    for(int i = 0; i < subTargets.length; ++i){
      subTargets[i] = segments[i].getRobotspaceEnd();
    }
    for(boolean b : enabledSubTargets){
      b = false;
    }

    target = segments[segments.length - 1].getRobotspaceEnd();
  }
  */

  @Override
  public void periodic() {
    //System.out.println(segments[0].angle - segments[0].encoderAngle);
    if(isInverseKinematicsEnabled){
      inverseKinematics();
      //subInverseKinematics();
      //updates the motors on the segments
      for(ManipulatorMoverSegment segment : segments){
        segment.update();
      }

      //for testing
      FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
    }else{
      for(ManipulatorMoverSegment segment : segments){
        segment.disabledUpdate();
      }
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
    for(int i = subTargets.length - 1; i >= 0; --i){
      if(enabledSubTargets[i]){

        segments[i].inverseKinematics(subTargets[i], anchor);
        for(ManipulatorMoverSegment segment : segments){
          segment.realign();
        }
        segments[0].forwardKinematics(anchor);

      }
    }
  }

  /**
   * incomplete
   * @return
   */
  public Vector3 getMeasuredFinalEndpoint(){
    //if simulation is running there are no encoders to measure from
    if(RobotMap.IS_SIMULATION_RUNNING){
      //return target;
      return segments[segments.length - 1].getRobotspaceEnd();
    }else{
      return segments[segments.length - 1].getMeasuredRobotspaceEnd();
    }
  }
  
  /**
   * incomplete
   * @return
   */
  public Vector3[] getMeasuredEndpoints(){
    Vector3[] endpoints = new Vector3[segments.length];
    //if simulation is running there are no encoders to measure from
    if(RobotMap.IS_SIMULATION_RUNNING){
      for(int i = 0; i < segments.length; ++i){
        endpoints[i] = segments[i].getRobotspaceEnd();
      }
    }else{
      for(int i = 0; i < segments.length; ++i){
        endpoints[i] = segments[i].getMeasuredRobotspaceEnd();
      }
    }

    return endpoints;
  }
  
  /*for testing*/
  public void updateKinematics(){
    inverseKinematics();
    subInverseKinematics();
    FileOutput.printManipulatorMoverState("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", this);
  }


}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.manipulator_mover;

import frc.robot.ExtraMath.*;


/**
 * @param length the segments length in arbitrary units
 * @param axis the axis of rotation
 * @param minAngle the lowest angle the segment can rotate
 * @param maxAngle the maximum angle the segment can rotate
 */
public class ManipulatorMoverSegment {
    
    static Vector3 initialOrientation = Vector3.up;
    /*private*/public Vector3 end;//localspace offset worldspace orientation
    public Vector3 end_worldspace = Vector3.zero;
    /*private*/public double angle = 0;//localspace
    double maxAngle = 0;//localspace
    double minAngle = 0;//localspace
    double length = 0;
    /*private*/public Vector3 axis;//localspace
    private Vector3 axis_worldspace;
    protected ManipulatorMoverSegment childSegment;
    protected ManipulatorMoverSegment parentSegment;
    
    public ManipulatorMoverSegment(double _length, Vector3 _axis, double _minAngle, double _maxAngle){
        length = _length;
        axis = _axis;
        maxAngle = _maxAngle;
        minAngle = _minAngle;
    }


    /*protected*/public void setChildSegment(ManipulatorMoverSegment segment){
        childSegment = segment;
        segment.parentSegment = this;
    }
    /*protected*/public void setParentSegment(ManipulatorMoverSegment segment){
        parentSegment = segment;
        segment.childSegment = this;
    }

    //removes rotation that occured off axis
    public/*protected*/ void realign(){
        end = Vector3.projectOnPlane(end, axis_worldspace).normalized().mult(length);

        if(parentSegment == null){
            angle = Vector3.signedAngle(initialOrientation, end, axis_worldspace);
        }else{
            angle = Vector3.signedAngle(parentSegment.end, end, axis_worldspace);
        }

        if(angle > maxAngle){
            angle = maxAngle;
        }
        if(angle < minAngle){
            angle = minAngle;
        }
    }

    public void forwardKinematics(){
        end = Vector3.rotate(initialOrientation, axis, angle).normalized().mult(length);
        end_worldspace = end;//worldspace end is the localspace end for the first segment
        axis_worldspace = axis;//worldspace axis is the localspace axis for the first segment

        if(childSegment != null){
            childSegment.forwardKinematics(end, end_worldspace.sub(Vector3.zero), new Vector3[]{axis_worldspace}, new double[]{angle});
        }
    }

    private void forwardKinematics(Vector3 base, Vector3 baseDirection, Vector3[] baseAxis, double[] baseAngle){
        Vector3[] nextAxis = new Vector3[baseAxis.length + 1];
        double[] nextAngle = new double[baseAngle.length + 1];

        //applies all previous rotations to determine the worldspace axis
        axis_worldspace = axis;
        for(int i = 0; i < baseAxis.length; ++i){
            axis_worldspace = Vector3.rotate(axis_worldspace, baseAxis[i], baseAngle[i]);
            nextAxis[i] = baseAxis[i];
            nextAngle[i] = baseAngle[i];  
        }
        nextAxis[nextAxis.length - 1] = axis_worldspace;
        nextAngle[nextAxis.length - 1] = angle;
        
        //calculates the end point
        end = Vector3.rotate(baseDirection, axis_worldspace, angle).normalized().mult(length);
        end_worldspace = end.add(base);

        //continues forward kinematics until complete
        if(childSegment != null){
            childSegment.forwardKinematics(end_worldspace, end_worldspace.sub(base), nextAxis, nextAngle);
        }
    }

    public void inverseKinematics(Vector3 target){        
        Vector3 nextTarget;
        if(parentSegment != null){
            nextTarget = parentSegment.end_worldspace.sub(target).normalized().mult(length).add(target);
            end = target.sub(parentSegment.end_worldspace).normalized().mult(length);
        }else{
            nextTarget = Vector3.zero.sub(target).normalized().mult(length).add(target);
            end = target.sub(Vector3.zero).normalized().mult(length);
        }
        
        end_worldspace = target;

        if(parentSegment != null){
            parentSegment.inverseKinematics(nextTarget);
        }
    }

    public void inverseKinematics(Vector3 target, boolean no){        
        Vector3 nextTarget;
        if(parentSegment != null){
            nextTarget = parentSegment.end_worldspace.sub(target).normalized().mult(length).add(target);
            end = target.sub(parentSegment.end_worldspace).normalized().mult(length);
        }else{
            nextTarget = Vector3.zero.sub(target).normalized().mult(length).add(target);
            end = target.sub(Vector3.zero).normalized().mult(length);
        }
        
        end_worldspace = target;

        if(parentSegment != null){
            //parentSegment.inverseKinematics(nextTarget);
        }
    }


}

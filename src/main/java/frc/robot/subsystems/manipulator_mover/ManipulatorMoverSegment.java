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
    /*private*/public Vector3 end;//localspace
    public Vector3 end_worldspace = Vector3.zero;
    /*private*/public double angle = 0;//localspace
    double maxAngle = 0;//localspace
    double minAngle = 0;//localspace
    double length = 0;
    /*private*/public Vector3 axis;//localspace
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
    private void realign(){
        end = Vector3.projectOnPlane(end, axis).normalized().mult(length);
    }

    public boolean forwardKinematics(){
        end = Vector3.rotate(initialOrientation, axis, angle);
        end_worldspace = end;
        if(childSegment == null){
            return false;
        }else{
            childSegment.forwardKinematics(end, end_worldspace.sub(Vector3.zero), new Vector3[]{axis}, new double[]{angle});
            return true;
        }

    }

    private boolean forwardKinematics(Vector3 base, Vector3 baseDirection, Vector3[] baseAxis, double[] baseAngle){
        Vector3[] nextAxis = new Vector3[baseAxis.length + 1];
        double[] nextAngle = new double[baseAngle.length + 1];

        Vector3 _axis = axis;
        for(int i = 0; i < baseAxis.length; ++i){
            _axis = Vector3.rotate(_axis, baseAxis[i], baseAngle[i]);
            nextAxis[i] = baseAxis[i];
            nextAngle[i] = baseAngle[i];  
        }
        nextAxis[nextAxis.length - 1] = _axis;
        nextAngle[nextAxis.length - 1] = angle;
        
        end = Vector3.rotate(baseDirection, _axis, angle);
        end_worldspace = end.add(base);


        if(childSegment == null){
            return false;
        }else{
            childSegment.forwardKinematics(end_worldspace, end_worldspace.sub(base), nextAxis, nextAngle);
            return true;
        }
    }
}

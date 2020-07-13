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
    
    static private Vector3 initialOrientation = Vector3.up;
    private Vector3 end;//localspace offset, robotspace orientation
    private Vector3 end_robotspace = Vector3.zero;
    protected/*private*/ double angle = 0;//localspace (degrees)
    double maxAngle = 0;//localspace
    double minAngle = 0;//localspace
    double length = 0;
    private Vector3 axis;//localspace (this does not change)
    private Vector3 axis_robotspace;

    protected ManipulatorMoverSegment childSegment;
    protected ManipulatorMoverSegment parentSegment;

    protected double encoderAngle;
    private Vector3 realAxis_robotspace;
    private Vector3 realEnd;
    private Vector3 realEnd_robotspace = Vector3.zero;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    protected double motorOutput = 0;
    private double error = 0;
    private double pastError = 0;
    private double sumError = 0;
    private double deltaError = 0;
    private double integralActivationThreshold = 0;//when the error is smaller than this threshold the i component is active
    
    public ManipulatorMoverSegment(double _length, Vector3 _axis, double _minAngle, double _maxAngle){
        length = _length;
        axis = _axis;
        maxAngle = _maxAngle;
        minAngle = _minAngle;
    }


    protected void setChildSegment(ManipulatorMoverSegment segment){
        childSegment = segment;
        segment.parentSegment = this;
    }
    protected void setParentSegment(ManipulatorMoverSegment segment){
        parentSegment = segment;
        segment.childSegment = this;
    }


    protected void update(){
        updateEncoder();
        pastError = error;
        error = angle - encoderAngle;
        deltaError = error - pastError;
        if(Math.abs(error) < integralActivationThreshold){
            sumError += error;
        }else{
            sumError = 0;
        }

        motorOutput = (kP * error) + (kI * sumError) + (kD * deltaError) + (Math.signum(error) * kFF);
        updateMotor();
    }

    protected void disabledUpdate(){
        updateEncoder();
        motorOutput = 0;
        updateMotor();
    }
    /**
     * Needs to be remade for different motors and encoders
     */
    protected void updateEncoder(){
        encoderAngle = 0;
    }

    protected void updateMotor(){

    }

    /*protected?*/public void setP(double value){
        kP = value;
    }

    /*protected?*/public void setI(double value){
        kI = value;
    }

    /*protected?*/public void setD(double value){
        kD = value;
    }

    /*protected?*/public void setFF(double value){
        kFF = value;
    }
    //
    //
    //Inverse kinematics stuff
    //
    //

    /**
     * removes rotation that occured off axis
     */
    protected void realign(){
        end = Vector3.projectOnPlane(end, axis_robotspace).normalized().mult(length);

        if(parentSegment == null){
            angle = Vector3.signedAngle(initialOrientation, end, axis_robotspace);
        }else{
            angle = Vector3.signedAngle(parentSegment.end, end, axis_robotspace);
        }

        if(angle > maxAngle){
            angle = maxAngle;
        }
        if(angle < minAngle){
            angle = minAngle;
        }
    }

    /**
     * Calculates the position of all child segments based on their angles and lengths.
     * @param anchor the starting location of the first segment 
     */
    protected void forwardKinematics(Vector3 anchor){
        end = Vector3.rotate(initialOrientation, axis, angle).normalized().mult(length);
        end_robotspace = end.add(anchor);//worldspace end is the localspace end for the first segment
        axis_robotspace = axis;//worldspace axis is the localspace axis for the first segment

        if(childSegment != null){
            childSegment.forwardKinematics(end_robotspace, end, new Vector3[]{axis_robotspace}, new double[]{angle});
        }
    }

    //the recusive part of the forward kinematics algorithm
    private void forwardKinematics(Vector3 base, Vector3 baseDirection, Vector3[] baseAxis, double[] baseAngle){
        Vector3[] nextAxis = new Vector3[baseAxis.length + 1];
        double[] nextAngle = new double[baseAngle.length + 1];

        //applies all previous rotations to determine the worldspace axis
        axis_robotspace = axis;
        for(int i = 0; i < baseAxis.length; ++i){
            axis_robotspace = Vector3.rotate(axis_robotspace, baseAxis[i], baseAngle[i]);
            nextAxis[i] = baseAxis[i];
            nextAngle[i] = baseAngle[i];  
        }
        nextAxis[nextAxis.length - 1] = axis_robotspace;
        nextAngle[nextAxis.length - 1] = angle;
        
        //calculates the end point
        end = Vector3.rotate(baseDirection, axis_robotspace, angle).normalized().mult(length);
        end_robotspace = end.add(base);

        //continues forward kinematics until complete
        if(childSegment != null){
            childSegment.forwardKinematics(end_robotspace, end, nextAxis, nextAngle);
        }
    }

    protected void measuredForwardKinematics(Vector3 anchor){
        realEnd = Vector3.rotate(initialOrientation, axis, encoderAngle).normalized().mult(length);
        realEnd_robotspace = realEnd.add(anchor);//robotspace end is the localspace end for the first segment
        realAxis_robotspace = axis;//robotspace axis is the localspace axis for the first segment

        if(childSegment != null){
            childSegment.measuredForwardKinematics(realEnd_robotspace, realEnd, new Vector3[]{realAxis_robotspace}, new double[]{encoderAngle});
        }
    }

    //the recusive part of the forward kinematics algorithm
    private void measuredForwardKinematics(Vector3 base, Vector3 baseDirection, Vector3[] baseAxis, double[] baseAngle){
        Vector3[] nextAxis = new Vector3[baseAxis.length + 1];
        double[] nextAngle = new double[baseAngle.length + 1];

        //applies all previous rotations to determine the worldspace axis
        realAxis_robotspace = axis;
        for(int i = 0; i < baseAxis.length; ++i){
            realAxis_robotspace = Vector3.rotate(realAxis_robotspace, baseAxis[i], baseAngle[i]);
            nextAxis[i] = baseAxis[i];
            nextAngle[i] = baseAngle[i];  
        }
        nextAxis[nextAxis.length - 1] = realAxis_robotspace;
        nextAngle[nextAxis.length - 1] = encoderAngle;
        
        //calculates the end point
        realEnd = Vector3.rotate(baseDirection, realAxis_robotspace, encoderAngle).normalized().mult(length);
        realEnd_robotspace = end.add(base);

        //continues forward kinematics until complete
        if(childSegment != null){
            childSegment.measuredForwardKinematics(realEnd_robotspace, realEnd, nextAxis, nextAngle);
        }
    }


    /**
     * A recursive inverse kinematics function
     * It updates all parent segments of the segment this function is called from.
     * @param target the desired end point
     * @param anchor the starting location of the first segment
     */
    protected void inverseKinematics(Vector3 target, Vector3 anchor){   
        Vector3 nextTarget;
        if(parentSegment != null){
            nextTarget = parentSegment.end_robotspace.sub(target).normalized().mult(length).add(target);
            end = target.sub(parentSegment.end_robotspace).normalized().mult(length);
        }else{
            nextTarget = anchor.sub(target).normalized().mult(length).add(target);
            end = target.sub(anchor).normalized().mult(length);
        }
        
        end_robotspace = target;

        if(parentSegment != null){
            parentSegment.inverseKinematics(nextTarget, anchor);
        }
    }

    public Vector3 getRobotspaceEnd(){
        return end_robotspace;
    }

    public Vector3 getMeasuredRobotspaceEnd(){
        return realEnd_robotspace;
    }


}

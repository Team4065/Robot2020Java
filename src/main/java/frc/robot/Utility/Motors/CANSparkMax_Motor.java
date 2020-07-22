/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class CANSparkMax_Motor extends Motor {


    CANSparkMax motor;
    CANPIDController motorPID;
    CANEncoder encoder;

    /**
     * 
     * @param CAN_ID
     * @param motorType brushless of brushed
     */
    public CANSparkMax_Motor(int CAN_ID, MotorType motorType){
        motor = new CANSparkMax(CAN_ID, motorType);
        motorPID = motor.getPIDController();
        encoder = motor.getEncoder();
    }

    /**
     * This sets the percent speed of the motor
     * @param value The percent output ranging from -1 to 1
     * @param controlMode This is non-functional on a PWM motor
     */
    @Override
    public void set(double value, ControlMode controlMode){
        switch(controlMode){
            case Percent:
                motor.set(value);
                break;

            case Velocity:
                motorPID.setReference(value, ControlType.kVelocity);
                break;

            case Position:
                motorPID.setReference(value, ControlType.kPosition);
                break;

            default:
                motor.set(value);
                break;
        }
        
    }

    @Override
    public void set(double value){
        motor.set(value);
    }

    @Override
    public double getRotations(){
        return encoder.getPosition();
    }

    @Override
    public double getSpeed(){
        return encoder.getVelocity() / 60;
    }
}

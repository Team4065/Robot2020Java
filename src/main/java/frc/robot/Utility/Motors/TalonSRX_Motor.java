/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

/**
 * Add your docs here.
 */
public class TalonSRX_Motor extends Motor {

    WPI_TalonSRX motor;
    boolean hasEncoder = false;

    public TalonSRX_Motor(int CAN_ID){
        motor = new WPI_TalonSRX(CAN_ID);
    }

    public TalonSRX_Motor(int CAN_ID, boolean _hasEncoder){
        motor = new WPI_TalonSRX(CAN_ID);
        if(_hasEncoder){
            hasEncoder = true;
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        }
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
                motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                break;

            case Velocity:
                motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, value);
                break;
            
            case Position:
                motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, value);
                break;
        }
        
    }

    @Override
    public void set(double value){
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
    }

    @Override
    public double getRotations(){
        if(hasEncoder){
            return (double)motor.getSelectedSensorPosition() / 4096;//4096 is the counts per revolution of the magnetic encoder
        }else{
            return Double.NaN;
        }
    }

    @Override
    public double getSpeed(){
        if(hasEncoder){
            return (double)motor.getSelectedSensorVelocity() / 4096 * 10;//4096 is the counts per revolution of the magnetic encoder. The 10 converts the measurement to rotations per second.
        }else{
            return Double.NaN;
        }
    }
}

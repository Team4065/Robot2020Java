/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

/**
 * Add your docs here.
 */
public class TalonSRX_Motor extends Motor {


    public TalonSRX_Motor(int CAN_ID){

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
                break;

            case Velocity:
                break;
            
            case Position:
                break;
        }
        
    }

    @Override
    public void set(double value){
        motor.setSpeed(value);
    }

    @Override
    public double getRotations(){
        if(encoder == null){
            return Double.NaN;
        }else{
            return (double)encoder.get() / cpm;
        }
    }
}

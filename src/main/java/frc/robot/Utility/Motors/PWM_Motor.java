/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class PWM_Motor extends Motor {

    int PWMPort;
    PWM motor;
    Encoder encoder = null;
    double cpm;//countsPerRevolution

    /**
     * 
     * @param PWM_Port The PWM port that the motor is connected to
     * @param encoderPort1 The ID of one of the encoder's digital I/O pins
     * @param encoderPort2 The ID of the encoder's other digital I/O pin
     * @param countsPerRevolution The number of counts per revolution of the encoder
     */
    public PWM_Motor(int PWM_Port, int encoderPort1, int encoderPort2, double countsPerRevolution){
        PWMPort = PWM_Port;
        motor = new PWM(PWM_Port);
        encoder = new Encoder(encoderPort1, encoderPort2);
        cpm = countsPerRevolution;
        encoder.setDistancePerPulse(1);
    }

    /**
     * 
     * @param PWM_Port The PWM port that the motor is connected to
     */
    public PWM_Motor(int PWM_Port){
        PWMPort = PWM_Port;
    }

    /**
     * This sets the percent speed of the motor
     * @param value The percent output ranging from -1 to 1
     * @param controlMode This is non-functional on a PWM motor
     */
    @Override
    public void set(double value, ControlMode controlMode){
        if(controlMode == ControlMode.Percent){
                set(value);
        }else{
            System.out.println("A PWM motor only accepts percent output control. Setting output to 0.");
            set(0);//safety
        }

        
    }

    @Override
    public void set(double value){
        if(inversion)
            motor.setSpeed(-value);
        else
            motor.setSpeed(value);
    }

    @Override
    public double getRotations(){
        if(encoder == null){
            return Double.NaN;
        }else{
            if(inversion)
                return -(double)encoder.get() / cpm;
            else
                return (double)encoder.get() / cpm;
        }
    }

    /**
     * Returns the rotations per second the motor.
     * PWM version might not work.
     * @return
     */
    @Override
    public double getSpeed(){
        if(encoder == null){
            return Double.NaN;
        }else{
            if(inversion)
                return -encoder.getRate() / cpm;
            else
                return encoder.getRate() / cpm;
        }
    }
}

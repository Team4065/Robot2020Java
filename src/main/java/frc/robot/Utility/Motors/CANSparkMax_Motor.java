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
public class CANSparkMax_Motor extends PID_Motor {


    CANSparkMax m_motor;
    CANPIDController m_motorPID;
    CANEncoder m_encoder;

    /**
     * 
     * @param CAN_ID
     * @param motorType brushless of brushed
     */
    public CANSparkMax_Motor(int CAN_ID, MotorType motorType){
        m_motor = new CANSparkMax(CAN_ID, motorType);
        m_motorPID = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
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
                m_motor.set(value);
                break;

            case Velocity:
                m_motorPID.setReference(value, ControlType.kVelocity);
                break;

            case Position:
                m_motorPID.setReference(value, ControlType.kPosition);
                break;

            default:
                m_motor.set(value);
                break;
        }
        
    }

    @Override
    public void set(double value){
        m_motor.set(value);
    }

    @Override
    public double getRotations(){
        return m_encoder.getPosition();
    }

    @Override
    public double getSpeed(){
        return m_encoder.getVelocity() / 60;
    }

     //Sets kP_velocity and updates the motorcontrollers
  @Override
  public void SetP_velocity(double value){
    m_kP_velocity = value;
    m_motorPID.setP(value, 0/*The 0 selects the PID configuration for velocity to be altered.*/);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_velocity(double value){
    m_kI_velocity = value;
    m_motorPID.setI(value, 0);
  }
  //Sets kD_velocity and updates the motorcontrollers
  @Override
  public void SetD_velocity(double value){
    m_kD_velocity = value;
    m_motorPID.setD(value, 0);
  }
  //Sets kFF_velocity and updates the motorcontrollers
  @Override
  public void SetFF_velocity(double value){
    m_kFF_velocity = value;
    m_motorPID.setFF(value, 0);
  }

  //Sets kP_position and updates the motorcontrollers
  @Override
  public void SetP_position(double value){
    m_kP_position = value;
    m_motorPID.setP(value, 1/*The 1 selects the PID configuration for position to be altered.*/);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_position(double value){
    m_kI_position = value;
    m_motorPID.setI(value, 1);
  }
  //Sets kD_position and updates the motorcontrollers
  @Override
  public void SetD_position(double value){
    m_kD_position = value;
    m_motorPID.setD(value, 1);
  }
  //Sets kF_position and updates the motorcontrollers
  @Override
  public void SetFF_position(double value){
    m_kFF_position = value;
    m_motorPID.setFF(value, 1);
  }

  @Override
  public void setInverted(boolean value){
    inversion = value;
    m_motor.setInverted(inversion);//Does noting if the motor is a slave
  }

  public void follow(CANSparkMax_Motor master, boolean opposeMaster){
    m_motor.follow(master.m_motor, opposeMaster);
  }
}

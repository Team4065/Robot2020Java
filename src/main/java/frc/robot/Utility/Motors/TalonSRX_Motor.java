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
public class TalonSRX_Motor extends PID_Motor {

  WPI_TalonSRX m_motor;
  boolean m_hasEncoder = false;

  public TalonSRX_Motor(int CAN_ID){
      m_motor = new WPI_TalonSRX(CAN_ID);
  }

  public TalonSRX_Motor(int CAN_ID, boolean hasEncoder){
      m_motor = new WPI_TalonSRX(CAN_ID);
      if(hasEncoder){
          m_hasEncoder = true;
          m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
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
              m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
              break;
          case Velocity:
              m_motor.selectProfileSlot(0, 0);
              m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, value);
              break;
          
          case Position:
              m_motor.selectProfileSlot(1, 0);
              m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, value);
              break;
          default:
              m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
              break;
      }
      
  }

  @Override
  public void set(double value){
      m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
  }

  @Override
  public double getRotations(){
      if(m_hasEncoder){
          return (double)m_motor.getSelectedSensorPosition() / 4096;//4096 is the counts per revolution of the magnetic encoder
      }else{
          return Double.NaN;
      }
  }

  @Override
  public double getSpeed(){
      if(m_hasEncoder){
          return (double)m_motor.getSelectedSensorVelocity() / 4096 * 10;//4096 is the counts per revolution of the magnetic encoder. The 10 converts the measurement to rotations per second.
      }else{
          return Double.NaN;
      }
  }
  
  //Sets kP_velocity and updates the motorcontrollers
  @Override
  public void SetP_velocity(double value){
    m_kP_velocity = value;
    m_motor.config_kP(0/*The 0 selects the PID configuration for velocity to be altered.*/, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_velocity(double value){
    m_kI_velocity = value;
    m_motor.config_kI(0, value);
  }
  //Sets kD_velocity and updates the motorcontrollers
  @Override
  public void SetD_velocity(double value){
    m_kD_velocity = value;
    m_motor.config_kD(0, value);
  }
  //Sets kFF_velocity and updates the motorcontrollers
  @Override
  public void SetFF_velocity(double value){
    m_kFF_velocity = value;
    m_motor.config_kF(0, value);
  }

  //Sets kP_position and updates the motorcontrollers
   @Override
  public void SetP_position(double value){
    m_kP_position = value;
    m_motor.config_kP(1/*The 1 selects the PID configuration for position to be altered.*/, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_position(double value){
    m_kI_position = value;
    m_motor.config_kI(1, value);
  }
  //Sets kD_position and updates the motorcontrollers
  @Override
  public void SetD_position(double value){
    m_kD_position = value;
    m_motor.config_kD(1, value);
  }
  //Sets kF_position and updates the motorcontrollers
  @Override
  public void SetFF_position(double value){
    m_kFF_position = value;
    m_motor.config_kF(1, value);
  }
}

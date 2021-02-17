// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSRX_DifferentialDrivetrain extends DifferentialDrivetrain {
  WPI_TalonSRX m_leftMaster, m_rightMaster;
  BaseMotorController[] m_leftSlaves, m_rightSlaves;

  /** Creates a new TalonSRX_DifferentialDrivetrain. */
  public TalonSRX_DifferentialDrivetrain(double wheelDiameter, WPI_TalonSRX leftMaster, WPI_TalonSRX rightMaster, BaseMotorController[] leftSlaves, BaseMotorController[] rightSlaves) {
    super(wheelDiameter);
    
    m_leftMaster = leftMaster;
    m_rightMaster = rightMaster;
    m_leftSlaves = leftSlaves;
    m_rightSlaves = rightSlaves;

    m_leftMaster.setInverted(InvertType.None);
    m_rightMaster.setInverted(InvertType.InvertMotorOutput);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    for(BaseMotorController slave : m_leftSlaves){
      slave.follow(m_leftMaster);
      slave.setInverted(InvertType.FollowMaster);
    }

    for(BaseMotorController slave : m_rightSlaves){
      slave.follow(m_rightMaster);
      slave.setInverted(InvertType.FollowMaster);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  @Override
  protected void setLeftPercent(double percent){
    m_leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, percent);
  }
  
  @Override
  protected void setRightPercent(double percent){
    m_rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, percent);
  }

  @Override
  protected void setLeftVoltage(double voltage){
    m_leftMaster.setVoltage(voltage);
  }

  @Override
  protected void setRightVoltage(double voltage){
    m_rightMaster.setVoltage(voltage);
  }

  @Override
  protected void setLeftVelocity(double velocity){
    if(m_isFeedforwardConfigured){

      double voltage = m_leftFeedforward.calculate(velocity, velocity - getLeftVelocity());
      m_leftMaster.setVoltage(voltage);
    }else{
      setLeftPercent(0);
    }
  }

  @Override
  protected void setRightVelocity(double velocity){
    if(m_isFeedforwardConfigured){
      double voltage = m_rightFeedforward.calculate(velocity, velocity - getRightVelocity());
      m_rightMaster.setVoltage(voltage);
    }else{
      setRightPercent(0);
    }
  }

  /**
   * @return The velocity of the left side in meters per second
   */
  @Override
  public double getLeftVelocity(){
    return -(double)m_leftMaster.getSelectedSensorVelocity() * 10.0 / 4096.0 * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The velocity of the right side in meters per second
   */
  @Override
  public double getRightVelocity(){
    return -(double)m_rightMaster.getSelectedSensorVelocity() * 10.0 / 4096.0 * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The distance the left side has traveled in meters.
   */
  @Override
  public double getLeftPosition(){
    return -(double)m_leftMaster.getSelectedSensorPosition() / 4096.0 * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The distance the right side has traveled in meters.
   */
  @Override
  public double getRightPosition(){
    return -(double)m_rightMaster.getSelectedSensorPosition() / 4096.0 * m_wheelDiameter * Math.PI;
  }

}

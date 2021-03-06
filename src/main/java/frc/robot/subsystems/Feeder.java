/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;

public class Feeder extends SubsystemBase {
  Motor m_feederMotor = new Motor(Constants.FEEDER_MOTOR_ID, Motor.MotorType.CANSparkMax);
  Motor m_serializerMotor = new Motor(Constants.SERIALIZER_MOTOR_ID, Motor.MotorType.CANSparkMax);
  /**
   * Creates a new Feeder.
   */
  public Feeder() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void onFeeder() {
    m_feederMotor.set(Motor.ControlMode.PercentOutput, 1);
  }

  public void offFeeder() {
    m_feederMotor.set(Motor.ControlMode.PercentOutput, 0);
  }

  public void onSerializer() {
    m_serializerMotor.set(Motor.ControlMode.PercentOutput, 1);
  }

  public void offSerializer() {
    m_serializerMotor.set(Motor.ControlMode.PercentOutput, 0);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motors.CANSparkMax_Motor;

public class Feeder extends SubsystemBase {
  CANSparkMax_Motor m_feederMotor = new CANSparkMax_Motor(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax_Motor m_serializerMotor = new CANSparkMax_Motor(Constants.SERIALIZER_MOTOR_ID, MotorType.kBrushless);
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
    m_feederMotor.set(1);
  }

  public void offFeeder() {
    m_feederMotor.set(0);
  }

  public void onSerializer() {
    m_feederMotor.set(1);
  }

  public void offSerializer() {
    m_feederMotor.set(0);
  }
}

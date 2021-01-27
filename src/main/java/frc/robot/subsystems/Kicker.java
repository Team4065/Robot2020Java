/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motors.CANSparkMax_Motor;

public class Kicker extends SubsystemBase {
  CANSparkMax_Motor m_kickerMotor = new CANSparkMax_Motor(Constants.KICKER_MOTOR_ID, MotorType.kBrushless);




  /**
   * Creates a new Kicker.
   */
  public Kicker() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void on() {
    m_kickerMotor.set(1);
  }

  public void off() {
    m_kickerMotor.set(0);
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.ControlMode;
import frc.robot.Utility.Motor.MotorType;

public class Feeder extends SubsystemBase {
  private Motor m_kicker = new Motor(Constants.KICKER, MotorType.CANSparkMax);
  private Motor m_feeder = new Motor(Constants.FEEDER, MotorType.CANSparkMax);

  /** Creates a new Feeder. */
  public Feeder() {
    m_kicker.enableBrakeMode(true);
    m_feeder.enableBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableKicker(){
    m_kicker.set(ControlMode.PercentOutput, 1);
  }

  public void disableKicker(){
    m_kicker.set(ControlMode.PercentOutput, 0);
  }

  public void enableFeeder(){
    m_feeder.set(ControlMode.PercentOutput, 1);
  }

  public void disableFeeder(){
    m_feeder.set(ControlMode.PercentOutput, 0);
  }
}

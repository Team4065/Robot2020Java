// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.ControlMode;
import frc.robot.Utility.Motor.MotorType;

public class Conveyor extends SubsystemBase {
  private Motor m_motor = new Motor(Constants.CONVEYOR, MotorType.CANSparkMax);
  /** Creates a new Conveyor. */
  public Conveyor() {
    m_motor.enableBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void in(){
    m_motor.set(ControlMode.PercentOutput, 1);
  }

  public void out(){
    m_motor.set(ControlMode.PercentOutput, -1);
  }

  public void stop(){
    m_motor.set(ControlMode.PercentOutput, 0);
  }
}

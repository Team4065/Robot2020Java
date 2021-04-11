// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.ControlMode;
import frc.robot.Utility.Motor.MotorType;

public class Flywheel extends SubsystemBase {
  private Motor m_left = new Motor(Constants.FLYWHEEL_LEFT, MotorType.TalonFX);
  private Motor m_right = new Motor(Constants.FLYWHEEL_RIGHT, MotorType.TalonFX);

  /** Creates a new Flywheel. */
  public Flywheel() {
    m_left.enableBrakeMode(false);
    m_right.enableBrakeMode(false);
    m_right.follow(m_left, true);
    m_left.configFeedforward(0.606918644699, 0.10519315078, -0.0000948594470576);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage(double voltage){
    m_left.set(ControlMode.Voltage, voltage);
  }

  public void setVelocity(double velocity){
    m_left.set(ControlMode.Velocity, velocity);
  }

  public double getVelocity(){
    return m_left.getVelocity();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.ControlMode;
import frc.robot.Utility.Motor.MotorType;

public class Intake extends SubsystemBase {
  private Motor m_motor = new Motor(Constants.INTAKE_MOTOR, MotorType.TalonSRX);
  private DoubleSolenoid m_solenoid = new DoubleSolenoid(Constants.INTAKE_DEPLOY, Constants.INTAKE_RETRACT);
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void suck(){
    m_motor.set(ControlMode.PercentOutput, 1);
  }

  public void spit(){
    m_motor.set(ControlMode.PercentOutput, -1);
  }

  public void hold(){
    m_motor.set(ControlMode.PercentOutput, 0);
  }

  public void deploy(){
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract(){
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}

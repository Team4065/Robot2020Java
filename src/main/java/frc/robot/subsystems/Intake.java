// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.ControlMode;
import frc.robot.Utility.Motor.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends SubsystemBase {
  Motor m_intakeMotor = new Motor(Constants.INTAKE_MOTOR_ID, MotorType.TalonSRX);
  DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(Constants.INTAKE_DEPLOY_ID, Constants.INTAKE_RETRACT_ID);
  

  
  /** Creates a new Intake. */
  public Intake() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy() {
    m_intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void in() {
    m_intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void out() {
    m_intakeMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}

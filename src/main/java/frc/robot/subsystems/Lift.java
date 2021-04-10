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

public class Lift extends SubsystemBase {
  private DoubleSolenoid m_solenoid = new DoubleSolenoid(Constants.LIFT_UP, Constants.LIFT_DOWN);
  private Motor m_liftLeft = new Motor(Constants.LEFT_LIFT_MOTOR, MotorType.CANSparkMax);
  private Motor m_liftRight = new Motor(Constants.RIGHT_LIFT_MOTOR, MotorType.CANSparkMax);

  /** Creates a new Lift. */
  public Lift() {
    m_liftLeft.setInverted(false);
    m_liftLeft.resetEncoder();
    m_liftRight.follow(m_liftLeft);
    m_liftLeft.enableBrakeMode(true);
    m_liftRight.enableBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_liftLeft.getVelocity() > 0 && this.getHeight() >= 0.9){
      m_liftLeft.set(ControlMode.PercentOutput, 0);
    }

    if(m_liftLeft.getVelocity() < 0 && this.getHeight() <= 0.0){
      m_liftLeft.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * 
   * @return The percent height of the lift
   */
  public double getHeight(){
    return m_liftLeft.getPosition() / Constants.MAX_LIFT_HEIGHT;
  }

  public double getDeltaHeight(){
    return m_liftLeft.getVelocity() / Constants.MAX_LIFT_HEIGHT;
  }

  public void moveUp(){
    if(this.getHeight() < 0.9){
      m_liftLeft.set(ControlMode.PercentOutput, 1);
    }else{
      m_liftLeft.set(ControlMode.PercentOutput, 0);
    }
  }

  public void moveDown(){
    if(this.getHeight() > 0){
      m_liftLeft.set(ControlMode.PercentOutput, -1);
    }else{
      m_liftLeft.set(ControlMode.PercentOutput, 0);
    }
  }

  public void stop(){
    m_liftLeft.set(ControlMode.PercentOutput, 0);
  }

  public void deploy(){
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract(){
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}

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

public class Shooter extends SubsystemBase {
  Motor m_leftMaster = new Motor(Constants.SHOOTER_LEFT_MASTER_ID, Motor.MotorType.TalonFX);
  Motor m_rightSlave = new Motor(Constants.SHOOTER_RIGHT_SLAVE_ID, Motor.MotorType.TalonFX);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_leftMaster.configFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);

    m_rightSlave.follow(m_leftMaster, true);
    m_leftMaster.setInverted(false);

    
    m_leftMaster.enableBrakeMode(false);
    m_rightSlave.enableBrakeMode(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void on(double RPM) {
    m_leftMaster.set(Motor.ControlMode.Velocity, RPM);
  }

  public void off() {
    m_leftMaster.set(Motor.ControlMode.PercentOutput, 0);
  }
}

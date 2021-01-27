/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motors.Motor;
import frc.robot.Utility.Motors.TalonFX_Motor;

public class Shooter extends SubsystemBase {
  TalonFX_Motor m_leftMaster = new TalonFX_Motor(Constants.SHOOTER_LEFT_MASTER_ID);
  TalonFX_Motor m_rightSlave = new TalonFX_Motor(Constants.SHOOTER_RIGHT_SLAVE_ID);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_leftMaster.SetP_velocity(Constants.SHOOTER_KP);
    m_leftMaster.SetFF_velocity(Constants.SHOOTER_KFF);
    m_leftMaster.SetI_velocity(Constants.SHOOTER_KI);
    m_leftMaster.SetD_velocity(Constants.SHOOTER_KD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void on(double RPM) {
    m_leftMaster.set(RPM, Motor.ControlMode.Velocity);
  }

  public void off() {
    m_leftMaster.set(0);
  }
}

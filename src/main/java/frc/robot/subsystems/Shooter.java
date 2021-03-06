/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Motor;

public class Shooter extends SubsystemBase {
  Motor m_leftMaster = new Motor(Constants.SHOOTER_LEFT_MASTER_ID, "TalonFX");
  Motor m_rightSlave = new Motor(Constants.SHOOTER_RIGHT_SLAVE_ID, "TalonFX");
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    /*
    m_leftMaster.SetP_velocity(Constants.SHOOTER_KP);
    m_leftMaster.SetFF_velocity(Constants.SHOOTER_KFF);
    m_leftMaster.SetI_velocity(Constants.SHOOTER_KI);
    m_leftMaster.SetD_velocity(Constants.SHOOTER_KD);

    m_rightSlave.SetP_velocity(Constants.SHOOTER_KP);
    m_rightSlave.SetFF_velocity(Constants.SHOOTER_KFF);
    m_rightSlave.SetI_velocity(Constants.SHOOTER_KI);
    m_rightSlave.SetD_velocity(Constants.SHOOTER_KD);
    */

    //TODO configure shooter feedforward

    // m_leftMaster.ConfigFactoryDefault();
    // m_rightSlave.ConfigFactoryDefault();

    m_rightSlave.follow(m_leftMaster, false);
    m_leftMaster.setInverted(false);
    m_rightSlave.setInverted(true);

    /*
    m_leftMaster.setNeutralMode(NeutralMode.Coast);
    m_rightSlave.setNeutralMode(NeutralMode.Coast);
    */

    // motorcontrol::SupplyCurrentLimitConfiguration supply_config_ {
    //     true, constants::shooter::kMaxCurrentDraw.to<double>(),
    //     constants::shooter::kMaxCurrentDraw.to<double>(),
    //     constants::shooter::kCurrentLimitingTriggerTime.to<double>()
    // };

    // m_leftMaster.ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero);
    // m_leftMaster.ConfigSupplyCurrentLimit(supply_config_);
    // m_rightSlave.ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero);
    // m_rightSlave.ConfigSupplyCurrentLimit(supply_config_);

    // m_leftMaster.ConfigOpenloopRamp(1.8);
    // m_rightSlave.ConfigOpenloopRamp(1.8);

    // m_leftMaster.ConfigClosedloopRamp(1.8);
    // m_rightSlave.ConfigClosedloopRamp(1.8);

    // m_leftMaster.Config_kP(0, constants::shooter::kP);
    // m_leftMaster.Config_kI(0, 0);
    // m_leftMaster.Config_kD(0, constants::shooter::kD);
    // m_leftMaster.Config_kF(0, constants::shooter::kFF);
    // m_leftMaster.Config_kF(0, 0.05);

    // m_rightSlave.Config_kP(0, constants::shooter::kP);
    // m_rightSlave.Config_kI(0, 0);
    // m_rightSlave.Config_kD(0, constants::shooter::kD);
    // m_rightSlave.Config_kF(0, constants::shooter::kFF);
    // m_rightSlave.Config_kF(0, 0.05);

    // m_leftMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
    // m_rightSlave.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDrivetrain extends SubsystemBase {
  enum ControlMode{
    PercentOutput,
    Voltage,
    Velocity
  };

  protected double m_leftTarget;
  protected double m_rightTarget;
  protected double m_leftKS, m_rightKS, m_leftKV, m_rightKV, m_leftKA, m_rightKA;
  protected SimpleMotorFeedforward m_leftFeedforward, m_rightFeedForward;
  protected boolean m_isFeedforwardConfigured = false;
  protected ControlMode m_controlMode = ControlMode.PercentOutput;
  protected DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());

  /** Creates a new DifferentialDrivetrain. */
  public DifferentialDrivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(m_controlMode){
      case PercentOutput:
        setLeftPercent(m_leftTarget);
        setRightPercent(m_rightTarget);
        break;

      case Voltage:
        setLeftVoltage(m_leftTarget);
        setRightVoltage(m_rightTarget);
        break;

      case Velocity:
        setLeftVelocity(m_leftTarget);
        setRightVelocity(m_rightTarget);
        break;

      default:
        setLeftPercent(0);
        setRightPercent(0);
        break;
    }
  }

  /**
   * Configures the feedforward.
   * This must be done before velocity control may be used.
   * Values can be obtained from robot characterization.
   * @param kS
   * @param kV
   * @param kA
   */
  public void configFeedforward(double kS, double kV, double kA){
    m_isFeedforwardConfigured = true;

    m_leftKS = kS;
    m_rightKS = kS;

    m_leftKV = kV;
    m_rightKV = kV;

    m_leftKA = kA;
    m_rightKA = kA;
  }
  
  /**
   * Same as configFeedforward, but it configures the left and right sides separately.
   * Values need to be obtained manually.
   * @param left_kS
   * @param left_kV
   * @param left_kA
   * @param right_kS
   * @param right_kV
   * @param right_kA
   */
  public void configFeedforwardSided(double left_kS, double left_kV, double left_kA, double right_kS, double right_kV, double right_kA){
    m_isFeedforwardConfigured = true;

    m_leftKS = left_kS;
    m_rightKS = right_kS;

    m_leftKV = left_kV;
    m_rightKV = right_kV;

    m_leftKA = left_kA;
    m_rightKA = right_kA;
  }




  private void setLeftPercent(double percent){}
  
  private void setRightPercent(double percent){}

  private void setLeftVoltage(double voltage){}

  private void setRightVoltage(double voltage){}

  private void setLeftVelocity(double velocity){}

  private void setRightVelocity(double velocity){}


}

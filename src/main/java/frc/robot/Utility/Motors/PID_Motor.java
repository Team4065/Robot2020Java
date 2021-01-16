/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility.Motors;

/**
 * Add your docs here.
 */
public class PID_Motor extends Motor {

    protected double m_kP_velocity, m_kI_velocity, m_kD_velocity, m_kFF_velocity = 0;
    protected double m_kP_position, m_kI_position, m_kD_position, m_kFF_position = 0;

    protected double m_target = 0;
    protected double m_error, m_sumError, m_deltaError, m_pastError = 0;

    //Sets kP_velocity and updates the motor controllers
    public void SetP_velocity(double value){m_kP_velocity = value;}
    //Sets kD_velocity and updates the motor controllers
    public void SetI_velocity(double value){m_kI_velocity = value;}
    //Sets kD_velocity and updates the motor controllers
    public void SetD_velocity(double value){m_kD_velocity = value;}
    //Sets kFF_velocity and updates the motor controllers
    public void SetFF_velocity(double value){m_kFF_velocity = value;}

    //Sets kP_position and updates the motor controllers
    public void SetP_position(double value){m_kP_position = value;}
    //Sets kD_position and updates the motor controllers
    public void SetI_position(double value){m_kI_position = value;}
    //Sets kD_position and updates the motor controllers
    public void SetD_position(double value){m_kD_position = value;}
    //Sets kF_position and updates the motor controllers
    public void SetFF_position(double value){m_kFF_position = value;}
}

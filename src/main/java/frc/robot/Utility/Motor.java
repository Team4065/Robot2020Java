// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class Motor {
    public enum ControlMode {
        PercentOutput, Voltage, Velocity
    };

    WPI_TalonSRX m_talonSRX;
    WPI_TalonFX m_talonFX;
    WPI_VictorSPX m_victorSPX;
    CANSparkMax m_canSparkMax;
    CANEncoder m_canEncoder;

    String m_motorType;
    boolean m_isFeedforwardConfigured = false;
    SimpleMotorFeedforward m_feedforward;

    public Motor(int id, String motorType) {
        m_motorType = motorType;
        if (m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX") {
            m_talonSRX = new WPI_TalonSRX(id);
            m_talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        }

        if (m_motorType == "TalonFX" || m_motorType == "WPI_TalonFX") {
            m_talonFX = new WPI_TalonFX(id);
            m_talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }

        if (m_motorType == "VictorSPX" || m_motorType == "WPI_VictorSPX") {
            m_victorSPX = new WPI_VictorSPX(id);
        }

        if (m_motorType == "CANSparkMax") {
            m_canSparkMax = new CANSparkMax(id, MotorType.kBrushless);
            m_canEncoder = m_canSparkMax.getEncoder();
        }
    }

    public void set(ControlMode controlMode, double value) {
        if (m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX") {
            switch (controlMode) {
                case PercentOutput:
                    m_talonSRX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
                case Voltage:
                    m_talonSRX.setVoltage(value);
                    break;
                case Velocity:
                    m_talonSRX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                    break;
                default:
                    m_talonSRX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
            }
        }

        if (m_motorType == "TalonFX" || m_motorType == "WPI_TalonFX") {
            switch (controlMode) {
                case PercentOutput:
                    m_talonFX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
                case Voltage:
                    m_talonFX.setVoltage(value);
                    break;
                case Velocity:
                    m_talonFX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                    break;
                default:
                    m_talonFX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
            }
        }

        if (m_motorType == "VictorSPX" || m_motorType == "WPI_VictorSPX") {
            switch (controlMode) {
                case PercentOutput:
                    m_victorSPX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
                case Voltage:
                    m_victorSPX.setVoltage(value);
                    break;
                case Velocity:
                    m_victorSPX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                    break;
                default:
                    m_victorSPX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                    break;
            }
        }

        if (m_motorType == "CANSparkMax") {
            switch (controlMode) {
                case PercentOutput:
                    m_canSparkMax.set(value);
                    break;
                case Voltage:
                    m_canSparkMax.setVoltage(value);
                    break;
                case Velocity:
                    m_canSparkMax.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                    break;
                default:
                    m_canSparkMax.set(value);
                    break;
            }
        }
    }

    public double getVelocity() {
        if (m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX") {
            return (double) m_talonSRX.getSelectedSensorVelocity() / 4096.0 * 10.0;
        }

        if (m_motorType == "TalonFX" || m_motorType == "WPI_TalonFX") {
            return (double) m_talonSRX.getSelectedSensorVelocity() / 2048.0 * 10.0;
        }

        if (m_motorType == "CANSparkMax") {
            return m_canEncoder.getVelocity() / 60.0;
        }

        return Double.NaN;
    }

    public double getPosition() {
        if (m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX") {
            return (double) m_talonSRX.getSelectedSensorPosition() / 4096.0;
        }

        if (m_motorType == "TalonFX" || m_motorType == "WPI_TalonFX") {
            return (double) m_talonSRX.getSelectedSensorPosition() / 2048.0;
        }

        if (m_motorType == "CANSparkMax") {
            return m_canEncoder.getPosition();
        }

        return Double.NaN;
    }

    public void follow(Motor master, boolean opposeMaster) {
        boolean isSelfCTRE = m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX" || m_motorType == "TalonFX"
                || m_motorType == "WPI_TalonFX" || m_motorType == "VictorSPX" || m_motorType == "WPI_VictorSPX";
        boolean isMasterCTRE = master.m_motorType == "TalonSRX" || master.m_motorType == "WPI_TalonSRX"
                || master.m_motorType == "TalonFX" || master.m_motorType == "WPI_TalonFX"
                || master.m_motorType == "VictorSPX" || master.m_motorType == "WPI_VictorSPX";

        if (isSelfCTRE && isMasterCTRE) {
            boolean isMasterTalonSRX = master.m_motorType == "TalonSRX" || master.m_motorType == "WPI_TalonSRX";
            boolean isMasterTalonFX = master.m_motorType == "TalonFX" || master.m_motorType == "WPI_TalonFX";

            var masterMotor = (isMasterTalonSRX) ? master.m_talonSRX
                    : (isMasterTalonFX) ? master.m_talonFX : master.m_victorSPX;

            if (m_motorType == "TalonSRX" || m_motorType == "WPI_TalonSRX") {
                m_talonSRX.follow(masterMotor);
                m_talonSRX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }

            if (m_motorType == "TalonFX" || m_motorType == "WPI_TalonFX") {
                m_talonFX.follow(masterMotor);
                m_talonFX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }

            if (m_motorType == "VictorSPX" || m_motorType == "WPI_VictorSPX") {
                m_victorSPX.follow(masterMotor);
                m_victorSPX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }
            return;
        }

        if (m_motorType == "CANSparkMax" && master.m_motorType == "CANSparkMax") {
            m_canSparkMax.follow(master.m_canSparkMax, opposeMaster);
            return;
        }

        try {
            throw new Exception("Cannot follow that motor type.");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void follow(Motor master){
        follow(master, false);
    }

}



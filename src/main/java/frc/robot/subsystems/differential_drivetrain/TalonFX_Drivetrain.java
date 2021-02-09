/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.differential_drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import frc.robot.Constants;
import frc.robot.Utility.Gyro;

public class TalonFX_Drivetrain extends Drivetrain {
  
  WPI_VictorSPX leftMaster, rightMaster;
  WPI_VictorSPX[] leftSlaves, rightSlaves;

  public TalonFX_Drivetrain() {
    int leftMotorCount = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1;
    int rightMotorCount = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1;

    leftMaster = new WPI_VictorSPX(Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN);//this makes the master motors that the slaves follow(mimic).

    leftSlaves = new WPI_VictorSPX[leftMotorCount - 1];

    int leftSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
    for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      leftSlaves[leftSlaveCount] = new WPI_VictorSPX(i);
      leftSlaves[leftSlaveCount].follow(leftMaster);//this makes the slaves follow the master.
      leftSlaves[leftSlaveCount].setInverted(InvertType.FollowMaster);
      ++leftSlaveCount;
    }

    rightMaster = new WPI_VictorSPX(Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN);

    rightSlaves = new WPI_VictorSPX[rightMotorCount - 1];

    int rightSlaveCount = 0;//this exists so that the correct address in the rightSlaves array is accessed.
    for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      rightSlaves[rightSlaveCount] = new WPI_VictorSPX(i);
      rightSlaves[rightSlaveCount].follow(rightMaster);//this makes the slaves follow the master.
      rightSlaves[rightSlaveCount].setInverted(InvertType.FollowMaster);
      ++rightSlaveCount;
    }
  
    leftMaster.setInverted(Constants.DRIVETRAIN_INVERT_FORWARD);
    rightMaster.setInverted(!Constants.DRIVETRAIN_INVERT_FORWARD);

    leftMaster.config_kP(2, Constants.KP_DRIVE_VEL);
    leftMaster.config_kI(2, 0);
    leftMaster.config_kD(2, 0);
    leftMaster.config_kF(2, 0);

    leftMaster.configSelectedFeedbackCoefficient(1);
    rightMaster.configSelectedFeedbackCoefficient(1);

    m_odometry.resetPosition(new Pose2d(), Gyro.getRotation2d());
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    m_feedForward = new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_odometry.update(Gyro.getRotation2d(), ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * leftMaster.getSelectedSensorPosition() / 2048 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI, ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * rightMaster.getSelectedSensorPosition() / 2048 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI);

    //this if statement and its contents are needed to implement simulation mode
    if(Constants.IS_SIMULATION_RUNNING){
      mySimulationPeriodic();
    }else{
      //this is the normal code to be run
      switch(controlMode){
        case PERCENT:
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightTarget);
          break;
        
        case VELOCITY:
          var wheelSpeeds = getWheelSpeeds();
          leftMaster.setVoltage(m_feedForward.calculate(leftTarget, leftTarget - wheelSpeeds.leftMetersPerSecond));
          rightMaster.setVoltage(m_feedForward.calculate(rightTarget, rightTarget - wheelSpeeds.rightMetersPerSecond));
          break;

        case VOLTAGE:
          leftMaster.setVoltage(leftTarget);
          rightMaster.setVoltage(rightTarget);
          break;
        
        default:
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightTarget);
          break;
      }
    }
  }
   
  @Override
  public double[] getLeftOutputs() {
    double[] output = new double[leftSlaves.length + 1];

    output[0] = leftMaster.get();
    for(int i = 1; i < output.length; ++i){
      output[i] = leftSlaves[i].getBusVoltage() / 12;//hopefully this is correct
    }

    return output;
  }

  @Override
  public double[] getRightOutputs() {
    double[] output = new double[rightSlaves.length + 1];

    output[0] = rightMaster.get();
    for(int i = 1; i < output.length; ++i){
      output[i] = rightSlaves[i].getBusVoltage() / 12;//hopefully this is correct
    }

    return output;
  }

  
  //Ramsete code
  //Everything is in meters
  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      (double)leftMaster.getSelectedSensorVelocity() / 2048 * 10 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI,//units/100ms to rot/100ms to rps to m/s
      (double)rightMaster.getSelectedSensorVelocity() / 2048 * 10 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI
     );
  }

  @Override
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  //In meters
  @Override
  public double getAverageEncoderDistance() {
    return ((leftMaster.getSelectedSensorPosition(0) / 2048 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI) + 
      (rightMaster.getSelectedSensorPosition(0) / 2048 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI)) / 2;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
Simulation mode is not accurate for PID.
It it only accurate for percent output.

The number of motors per side is determined in Constants using the min and max ids for the left and right sides
*/
package frc.robot.subsystems.differential_drivetrain;

import frc.robot.Constants;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Gyro;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Drivetrain extends SubsystemBase {
  protected SimpleMotorFeedforward m_feedForward;

  protected  DifferentialDriveOdometry m_odometry;

  protected double leftTarget, rightTarget = 0;
  public Talon[] simulationMotors;

  public static enum ControlMode {
    PERCENT,
    VELOCITY,
    VOLTAGE
  }

  protected ControlMode controlMode = ControlMode.PERCENT;

  public Drivetrain() {
    int leftMotorCount = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1;
    int rightMotorCount = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1;

    simulationMotors = new Talon[leftMotorCount + rightMotorCount];
    
    int createdSimulationMotors = 0;
    for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      simulationMotors[createdSimulationMotors] = new Talon(i);
      ++createdSimulationMotors;
    }

    for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      simulationMotors[createdSimulationMotors] = new Talon(i);
      ++createdSimulationMotors;
    }

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftPosition(), getRightPosition());
  }

  public void setLeftTarget(double value){
    leftTarget = value;
  }

  public void setRightTarget(double value){
    rightTarget = value;
  }

  public double getLeftTarget(){
    return leftTarget;
  }

  public double getRightTarget(){
    return rightTarget;
  }

  public void setControlMode(ControlMode mode){
    controlMode = mode;
  }

  public ControlMode getControlMode(){
    return controlMode;
  }

  protected void mySimulationPeriodic(){
    int updatedMotors = 0;//this exists so that the correct address in the simulationMotors array is accessed.
    //updates the left motors
    for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      switch(controlMode){
        case PERCENT:
          simulationMotors[updatedMotors].set(leftTarget);
          break;
        case VELOCITY:
          simulationMotors[updatedMotors].setSpeed(leftTarget);
          break;
        case VOLTAGE:
          simulationMotors[updatedMotors].setVoltage(leftTarget);
          break;
        default:
         simulationMotors[updatedMotors].set(leftTarget);
          break;
      }
      ++updatedMotors;
    }

    //updates the right motors
    for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      switch(controlMode){
        case PERCENT:
          simulationMotors[updatedMotors].set(rightTarget);
          break;
        case VELOCITY:
          simulationMotors[updatedMotors].setSpeed(rightTarget);
          break;
  
        case VOLTAGE:
          simulationMotors[updatedMotors].setVoltage(rightTarget);
          break;
  
        default:
         simulationMotors[updatedMotors].set(rightTarget);
          break;
      }
      ++updatedMotors;
    }
  }

  public double[] getLeftOutputs(){
    return new double[]{};
  }

  public double[] getRightOutputs(){
    return new double[]{};
  }

  //Ramsete code

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds();
  }

  public double getHeading(){
    return Gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -Gyro.getRate();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  
  public void tankDriveMeterPerSecond(double leftVelocity, double rightVelocity){
    setControlMode(ControlMode.VELOCITY);
    setLeftTarget(leftVelocity);
    setRightTarget(rightVelocity);
  }

  public void tankDriveVolts(double left, double right){
    setControlMode(ControlMode.VOLTAGE);
    setLeftTarget(left);
    setRightTarget(right);
  }

  public void resetEncoders(){

  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, Gyro.getRotation2d());
  }

  //In meters
  public double getAverageEncoderDistance() {
    return 0;
  }

  public void zeroHeading() {
    Gyro.reset();
  }

  public double getLeftPosition(){
    return 0;
  }

  public double getRightPosition(){
    return 0;
  }
}

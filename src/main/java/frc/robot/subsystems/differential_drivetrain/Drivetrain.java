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

public class Drivetrain extends SubsystemBase {

  protected double kP_velocity, kI_velocity, kD_velocity, kFF_velocity = 0;
  protected double kP_position, kI_position, kD_position, kFF_position = 0;
  protected double kMaxVelocity_velocity, kMaxAcceleration_velocity = 0;
  protected double kMaxVelocity_position, kMaxAcceleration_position = 0;


  protected  DifferentialDriveOdometry m_odometry;

  protected double leftTarget, rightTarget = 0;
  public Talon[] simulationMotors;

  public static enum ControlMode {
    PERCENT,
    VELOCITY,
    POSITION,
    RAMSETE
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
  
        case POSITION:
          simulationMotors[updatedMotors].setPosition(leftTarget);
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
  
        case POSITION:
          simulationMotors[updatedMotors].setPosition(rightTarget);
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

  //Sets kP_velocity and updates the motor controllers
  public void SetP_velocity(double value){kP_velocity = value;}
  //Sets kD_velocity and updates the motor controllers
  public void SetI_velocity(double value){kI_velocity = value;}
  //Sets kD_velocity and updates the motor controllers
  public void SetD_velocity(double value){kD_velocity = value;}
  //Sets kFF_velocity and updates the motor controllers
  public void SetFF_velocity(double value){kFF_velocity = value;}

  //Sets kP_position and updates the motor controllers
  public void SetP_position(double value){kP_position = value;}
  //Sets kD_position and updates the motor controllers
  public void SetI_position(double value){kI_position = value;}
  //Sets kD_position and updates the motor controllers
  public void SetD_position(double value){kD_position = value;}
  //Sets kF_position and updates the motor controllers
  public void SetFF_position(double value){kFF_position = value;}

  //Sets the max velocity of the motor controllers
  public void SetMaxVelocity_velocity(double value){
    kMaxVelocity_velocity = value;
  }
  //Sets the max acceleration of the motor controllers
  public void SetMaxAcceleration_velocity(double value){
    kMaxAcceleration_velocity = value;
  }

  //Sets the max velocity of the motor controllers
  public void SetMaxVelocity_position(double value){
    kMaxVelocity_position = value;
  }
  //Sets the max acceleration of the motor controllers
  public void SetMaxAcceleration_position(double value){
    kMaxAcceleration_position = value;
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
}

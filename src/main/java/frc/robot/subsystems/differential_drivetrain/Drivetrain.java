/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
Simulation mode is not accurate for PID.
It it only accurate for percent output.

The number of motors per side is determined in RobotMap using the min and max ids for the left and right sides
*/
package frc.robot.subsystems.differential_drivetrain;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  protected double kP_velocity, kI_velocity, kD_velocity, kFF_velocity = 0;
  protected double kP_position, kI_position, kD_position, kFF_position = 0;

  protected double leftTarget, rightTarget = 0;
  protected Talon[] simulationMotors;

  public static enum ControlMode {
    PERCENT,
    VELOCITY,
    POSITION
  }

  protected ControlMode controlMode = ControlMode.PERCENT;

  public Drivetrain() {
    int leftMotorCount = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1;
    int rightMotorCount = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1;

    simulationMotors = new Talon[leftMotorCount + rightMotorCount];
    
    int createdSimulationMotors = 0;
    for(int i = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN; i <= RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      simulationMotors[createdSimulationMotors] = new Talon(i);
      ++createdSimulationMotors;
    }

    for(int i = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN; i <= RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      simulationMotors[createdSimulationMotors] = new Talon(i);
      ++createdSimulationMotors;
    }

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

  protected void simulationPeriodic(){
    int updatedMotors = 0;//this exists so that the correct address in the simulationMotors array is accessed.
    //updates the left motors
    for(int i = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN; i <= RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
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
    for(int i = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN; i <= RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
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

  //Sets kP_velocity and updates the motor controllers
  void SetP_velocity(double value){kP_velocity = value;}
  //Sets kD_velocity and updates the motor controllers
  void SetI_velocity(double value){kI_velocity = value;}
  //Sets kD_velocity and updates the motor controllers
  void SetD_velocity(double value){kD_velocity = value;}
  //Sets kFF_velocity and updates the motor controllers
  void SetFF_velocity(double value){kFF_velocity = value;}

  //Sets kP_position and updates the motor controllers
  void SetP_position(double value){kP_position = value;}
  //Sets kD_position and updates the motor controllers
  void SetI_position(double value){kI_position = value;}
  //Sets kD_position and updates the motor controllers
  void SetD_position(double value){kD_position = value;}
  //Sets kF_position and updates the motor controllers
  void SetFF_position(double value){kFF_position = value;}
}

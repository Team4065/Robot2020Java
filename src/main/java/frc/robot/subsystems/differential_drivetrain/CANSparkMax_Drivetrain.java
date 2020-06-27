/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.differential_drivetrain;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CANSparkMax_Drivetrain extends Drivetrain {

  private CANSparkMax leftMaster, rightMaster; //These are the motor controllers that act as masters for the drivetrain
  private CANPIDController leftPID, rightPID;  //These allow access to inbuilt closed loops in the motor controllers
  private CANSparkMax[] leftSlaves, rightSlaves;


  /**
   * Creates a new CANSparkMax_Drivetrain.
   */
  public CANSparkMax_Drivetrain(boolean invertForward, boolean isBrushless) {
    //Picks between brushed and brushless motors (ask the mechanical or electrical teams for that information)
    MotorType motorType = (isBrushless) ? MotorType.kBrushless : MotorType.kBrushed;

    leftMaster = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN, motorType);
    rightMaster = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN, motorType);

    leftPID = leftMaster.getPIDController();
    rightPID = rightMaster.getPIDController();

    leftSlaves = new CANSparkMax[RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - 1];
    rightSlaves = new CANSparkMax[RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - 1];

    int leftSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
    for(int i = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      leftSlaves[leftSlaveCount] = new CANSparkMax(i, motorType);//makes the slaves
      leftSlaves[leftSlaveCount].follow(leftMaster);//binds the slaves to the masters
      ++leftSlaveCount;
    }

    int rightSlaveCount = 0;//this exists so that the correct address in the rightSlaves array is accessed.
    for(int i = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      rightSlaves[rightSlaveCount] = new CANSparkMax(i, motorType);//makes the slaves
      rightSlaves[rightSlaveCount].follow(rightMaster);//binds the slaves to the masters
      ++rightSlaveCount;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotMap.IS_SIMULATION_RUNNING){
      simulationPeriodic();
    }else{
      switch(controlMode){
        case PERCENT:
          leftMaster.set(leftTarget);
          rightMaster.set(rightTarget);
          break;
  
        case VELOCITY:
          leftPID.setReference(leftTarget, ControlType.kVelocity, 0);
          rightPID.setReference(rightTarget, ControlType.kVelocity, 0);
          break;
  
        case POSITION:
          leftPID.setReference(leftTarget, ControlType.kPosition, 1);
          rightPID.setReference(rightTarget, ControlType.kPosition, 1);
          break;
  
        default:
          leftMaster.set(leftTarget);
          rightMaster.set(rightTarget);
          break;
      }
    }
  }

  //Sets kP_velocity and updates the motorcontrollers
  @Override
  public void SetP_velocity(double value){
    kP_velocity = value;
    leftPID.setP(value, 0/*The 0 selects the PID configuration for velocity to be altered.*/);
    rightPID.setP(value, 0);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_velocity(double value){
    kI_velocity = value;
    leftPID.setI(value, 0);
    rightPID.setI(value, 0);
  }
  //Sets kD_velocity and updates the motorcontrollers
  @Override
  public void SetD_velocity(double value){
    kD_velocity = value;
    leftPID.setD(value, 0);
    rightPID.setD(value, 0);
  }
  //Sets kFF_velocity and updates the motorcontrollers
  @Override
  public void SetFF_velocity(double value){
    kFF_velocity = value;
    leftPID.setFF(value, 0);
    rightPID.setFF(value, 0);
  }

  //Sets kP_position and updates the motorcontrollers
  @Override
  public void SetP_position(double value){
    kP_position = value;
    leftPID.setP(value, 1/*The 1 selects the PID configuration for position to be altered.*/);
    rightPID.setP(value, 1);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_position(double value){
    kI_position = value;
    leftPID.setI(value, 1);
    rightPID.setI(value, 1);
  }
  //Sets kD_position and updates the motorcontrollers
  @Override
  public void SetD_position(double value){
    kD_position = value;
    leftPID.setD(value, 1);
    rightPID.setD(value, 1);
  }
  //Sets kF_position and updates the motorcontrollers
  @Override
  public void SetFF_position(double value){
    kFF_position = value;
    leftPID.setFF(value, 1);
    rightPID.setFF(value, 1);
  }

  //Sets the max velocity of the motor controllers
  @Override
  public void SetMaxVelocity_velocity(double value){
    kMaxVelocity_velocity = value;
    leftPID.setSmartMotionMaxVelocity(value, 0);
    rightPID.setSmartMotionMaxVelocity(value, 0);
  }
  //Sets the max acceleration of the motor controllers
  @Override
  public void SetMaxAcceleration_velocity(double value){
    kMaxAcceleration_velocity = value;
    leftPID.setSmartMotionMaxAccel(value, 0);
    rightPID.setSmartMotionMaxAccel(value, 0);
  }

  //Sets the max velocity of the motor controllers
  @Override
  public void SetMaxVelocity_position(double value){
    kMaxVelocity_position = value;
    leftPID.setSmartMotionMaxVelocity(value, 1);
    rightPID.setSmartMotionMaxVelocity(value, 1);
  }
  //Sets the max acceleration of the motor controllers
  @Override
  public void SetMaxAcceleration_position(double value){
    kMaxAcceleration_position = value;
    leftPID.setSmartMotionMaxAccel(value, 1);
    rightPID.setSmartMotionMaxAccel(value, 1);
  }
}

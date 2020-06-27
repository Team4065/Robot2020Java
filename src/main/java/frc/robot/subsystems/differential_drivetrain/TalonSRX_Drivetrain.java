/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.differential_drivetrain;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * A drivetrain subsystem that uses TalonSRX as the motor controller. VictorSPX
 * can be used as slave controllers. CAN IDs range from 1 to 6. IDs 1 - 3: left
 * motors IDs 4 - 6: right motors
 * 
 * @param leftMotorCount     The number of motors on the left side of the
 *                           drivetrain.
 * @param rightMotorCount    The number of motors on the right side of the
 *                           drivetrain.
 * @param areSlavesVictorSPX True if the slaves are VictorSPXs.
 * @param invertForward      Inverts the forward direction of the drivetrain and
 *                           inverts encoders accordingly.
 * 
 */
public class TalonSRX_Drivetrain extends Drivetrain {

  WPI_TalonSRX leftMaster, rightMaster;
  BaseMotorController[] leftSlaves, rightSlaves;

  public TalonSRX_Drivetrain(boolean invertForward, boolean areSlavesVictorSPX) {
    
    int leftMotorCount = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1;
    int rightMotorCount = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1;


    leftMaster = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN);//this makes the master motors that the slaves follow(mimic).

    //Adds slaves of the appropriate type
    if(areSlavesVictorSPX){
      leftSlaves = new WPI_VictorSPX[leftMotorCount - 1];

      int leftSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
        leftSlaves[leftSlaveCount] = new WPI_VictorSPX(i);
        leftSlaves[leftSlaveCount].follow(leftMaster);//this makes the slaves follow the master.
        ++leftSlaveCount;
      }
    }else{
      leftSlaves = new WPI_TalonSRX[leftMotorCount - 1];

      int leftSlaveCount = 0;//this exists so that the correct address in the rightSlaves array is accessed.
      for(int i = RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
        leftSlaves[leftSlaveCount] = new WPI_TalonSRX(i);
        leftSlaves[leftSlaveCount].follow(leftMaster);//this makes the slaves follow the master.
        ++leftSlaveCount;
      }
    }
    

    rightMaster = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN);

    //Adds slaves of the appropriate type
    if(areSlavesVictorSPX){
      rightSlaves = new WPI_VictorSPX[rightMotorCount - 1];

      int rightSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
        rightSlaves[rightSlaveCount] = new WPI_VictorSPX(i);
        rightSlaves[rightSlaveCount].follow(rightMaster);//this makes the slaves follow the master.
        ++rightSlaveCount;
      }
    }else{
      rightSlaves = new WPI_TalonSRX[rightMotorCount - 1];

      int rightSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= RobotMap.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
        rightSlaves[rightSlaveCount] = new WPI_TalonSRX(i);
        rightSlaves[rightSlaveCount].follow(rightMaster);//this makes the slaves follow the master.
        ++rightSlaveCount;
      }
    }

    leftMaster.setInverted(!invertForward);
    rightMaster.setInverted(!invertForward);
  }
  public TalonSRX_Drivetrain(boolean invertForward){
    this(invertForward, false);
  }
  public TalonSRX_Drivetrain(){
    this(false, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //this if statement and its contents are needed to implement simulation mode
    if(RobotMap.IS_SIMULATION_RUNNING){
      simulationPeriodic();
    }else{
      //this is the normal code to be run
      switch(controlMode){
        case PERCENT:
          leftMaster.set(leftTarget);
          rightMaster.set(rightTarget);
          break;
        
        case VELOCITY:
          //selects the proper PID values
          leftMaster.selectProfileSlot(0, 0);
          rightMaster.selectProfileSlot(0, 0);
  
          //Updates the PID target
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, rightTarget);
          break;
  
        case POSITION:
          //selects the proper PID values
          leftMaster.selectProfileSlot(1, 0);
          rightMaster.selectProfileSlot(1, 0);
  
          //Updates the PID target
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, rightTarget);
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
  void SetP_velocity(double value){
    kP_velocity = value;
    leftMaster.config_kP(0/*The 0 selects the PID configuration for velocity to be altered.*/, value);
    rightMaster.config_kP(0, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  void SetI_velocity(double value){
    kI_velocity = value;
    leftMaster.config_kI(0, value);
    rightMaster.config_kI(0, value);
  }
  //Sets kD_velocity and updates the motorcontrollers
  @Override
  void SetD_velocity(double value){
    kD_velocity = value;
    leftMaster.config_kD(0, value);
    rightMaster.config_kD(0, value);
  }
  //Sets kFF_velocity and updates the motorcontrollers
  @Override
  void SetFF_velocity(double value){
    kFF_velocity = value;
    leftMaster.config_kF(0, value);
    rightMaster.config_kF(0, value);
  }

  //Sets kP_position and updates the motorcontrollers
   @Override
  void SetP_position(double value){
    kP_position = value;
    leftMaster.config_kP(1/*The 1 selects the PID configuration for position to be altered.*/, value);
    rightMaster.config_kP(1, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  void SetI_position(double value){
    kI_position = value;
    leftMaster.config_kI(1, value);
    rightMaster.config_kI(1, value);
  }
  //Sets kD_position and updates the motorcontrollers
  @Override
  void SetD_position(double value){
    kD_position = value;
    leftMaster.config_kD(1, value);
    rightMaster.config_kD(1, value);
  }
  //Sets kF_position and updates the motorcontrollers
  @Override
  void SetFF_position(double value){
    kFF_position = value;
    leftMaster.config_kF(1, value);
    rightMaster.config_kF(1, value);
  }
}

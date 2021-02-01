/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.differential_drivetrain;

import frc.robot.Robot;
import frc.robot.Utility.Gyro;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

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

  public TalonSRX_Drivetrain(boolean areSlavesVictorSPX) {
    
    int leftMotorCount = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1;
    int rightMotorCount = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1;


    leftMaster = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN);//this makes the master motors that the slaves follow(mimic).
    

    //Adds slaves of the appropriate type
    if(areSlavesVictorSPX){
      leftSlaves = new WPI_VictorSPX[leftMotorCount - 1];

      int leftSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
        leftSlaves[leftSlaveCount] = new WPI_VictorSPX(i);
        leftSlaves[leftSlaveCount].follow(leftMaster, FollowerType.PercentOutput);//this makes the slaves follow the master.
        leftSlaves[leftSlaveCount].setInverted(InvertType.FollowMaster);
        ++leftSlaveCount;
      }
    }else{
      leftSlaves = new WPI_TalonSRX[leftMotorCount - 1];

      int leftSlaveCount = 0;//this exists so that the correct address in the rightSlaves array is accessed.
      for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
        leftSlaves[leftSlaveCount] = new WPI_TalonSRX(i);
        leftSlaves[leftSlaveCount].follow(leftMaster, FollowerType.PercentOutput);//this makes the slaves follow the master.
        leftSlaves[leftSlaveCount].setInverted(InvertType.FollowMaster);
        ++leftSlaveCount;
      }
    }
    

    rightMaster = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN);
    

    //Adds slaves of the appropriate type
    if(areSlavesVictorSPX){
      rightSlaves = new WPI_VictorSPX[rightMotorCount - 1];

      int rightSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
        rightSlaves[rightSlaveCount] = new WPI_VictorSPX(i);
        rightSlaves[rightSlaveCount].follow(rightMaster, FollowerType.PercentOutput);//this makes the slaves follow the master.
        rightSlaves[rightSlaveCount].setInverted(InvertType.FollowMaster);
        ++rightSlaveCount;
      }
    }else{
      rightSlaves = new WPI_TalonSRX[rightMotorCount - 1];

      int rightSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
      for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
        rightSlaves[rightSlaveCount] = new WPI_TalonSRX(i);
        rightSlaves[rightSlaveCount].follow(rightMaster, FollowerType.PercentOutput);//this makes the slaves follow the master.
        rightSlaves[rightSlaveCount].setInverted(InvertType.FollowMaster);
        ++rightSlaveCount;
      }
    }

    leftMaster.setInverted(Constants.DRIVETRAIN_INVERT_FORWARD);
    rightMaster.setInverted(!Constants.DRIVETRAIN_INVERT_FORWARD);

    leftMaster.config_kP(2, Constants.KP_DRIVE_VEL);
    leftMaster.config_kI(2, 0);
    leftMaster.config_kD(2, 0);
    leftMaster.config_kF(2, 0);
  }
  public TalonSRX_Drivetrain(){
    this(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.print(this.leftMaster.getSelectedSensorVelocity() / 4096 * 10);
    System.out.print("       ");
    System.out.println(this.rightMaster.getSelectedSensorVelocity() / 4096 * 10);

    m_odometry.update(Gyro.getRotation2d(), leftMaster.getSelectedSensorPosition() / 4096 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI, rightMaster.getSelectedSensorPosition() / 4096 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI);

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

        case RAMSETE:
          //selects the proper PID values
          leftMaster.selectProfileSlot(2, 0);
          rightMaster.selectProfileSlot(2, 0);

          //Updates the PID target
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, rightTarget);
          break;
        
        default:
          leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftTarget);
          rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightTarget);
          break;
      }
    }
  }

  //Sets kP_velocity and updates the motorcontrollers
  @Override
  public void SetP_velocity(double value){
    kP_velocity = value;
    leftMaster.config_kP(0/*The 0 selects the PID configuration for velocity to be altered.*/, value);
    rightMaster.config_kP(0, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_velocity(double value){
    kI_velocity = value;
    leftMaster.config_kI(0, value);
    rightMaster.config_kI(0, value);
  }
  //Sets kD_velocity and updates the motorcontrollers
  @Override
  public void SetD_velocity(double value){
    kD_velocity = value;
    leftMaster.config_kD(0, value);
    rightMaster.config_kD(0, value);
  }
  //Sets kFF_velocity and updates the motorcontrollers
  @Override
  public void SetFF_velocity(double value){
    kFF_velocity = value;
    leftMaster.config_kF(0, value);
    rightMaster.config_kF(0, value);
  }

  //Sets kP_position and updates the motorcontrollers
   @Override
  public void SetP_position(double value){
    kP_position = value;
    leftMaster.config_kP(1/*The 1 selects the PID configuration for position to be altered.*/, value);
    rightMaster.config_kP(1, value);
  }
  //Sets kI_velocity and updates the motorcontrollers
  @Override
  public void SetI_position(double value){
    kI_position = value;
    leftMaster.config_kI(1, value);
    rightMaster.config_kI(1, value);
  }
  //Sets kD_position and updates the motorcontrollers
  @Override
  public void SetD_position(double value){
    kD_position = value;
    leftMaster.config_kD(1, value);
    rightMaster.config_kD(1, value);
  }
  //Sets kF_position and updates the motorcontrollers
  @Override
  public void SetFF_position(double value){
    kFF_position = value;
    leftMaster.config_kF(1, value);
    rightMaster.config_kF(1, value);
  }

  //Sets the max velocity of the motor controllers
  @Override
  public void SetMaxVelocity_velocity(double value){
    kMaxVelocity_velocity = value;
    throw new UnsupportedOperationException();
  }
  //Sets the max acceleration of the motor controllers
  @Override
  public void SetMaxAcceleration_velocity(double value){
    kMaxAcceleration_velocity = value;
    throw new UnsupportedOperationException();
  }

  //Sets the max velocity of the motor controllers
  @Override
  public void SetMaxVelocity_position(double value){
    kMaxVelocity_position = value;
    throw new UnsupportedOperationException();
  }
  //Sets the max acceleration of the motor controllers
  @Override
  public void SetMaxAcceleration_position(double value){
    kMaxAcceleration_position = value;
    throw new UnsupportedOperationException();
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
      (double)leftMaster.getSelectedSensorVelocity() / 4096 * 10 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI,//the times 10 brings it from per 100ms to 1000ms
      (double)rightMaster.getSelectedSensorVelocity() / 4096 * 10 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI
     );
  }

  @Override
  public void tankDriveMeterPerSecond(double leftVelocity, double rightVelocity){
    setControlMode(ControlMode.RAMSETE);
    setLeftTarget(leftVelocity / (Math.PI * Constants.ROBOT_WHEEL_DIAMETER) * 4096 / 10);//meters/second to rotations/second to units/100 milliseconds
    setRightTarget(leftVelocity / (Math.PI * Constants.ROBOT_WHEEL_DIAMETER) * 4096 / 10);
  }

  @Override
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  //In meters
  @Override
  public double getAverageEncoderDistance() {
    return ((leftMaster.getSelectedSensorPosition(0) / 4096 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI) + 
      (rightMaster.getSelectedSensorPosition(0) / 4096 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI)) / 2;
  }
}

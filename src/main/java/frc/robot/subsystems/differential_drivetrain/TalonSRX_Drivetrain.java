/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Try making a ramsete command using the WPI PID controller

package frc.robot.subsystems.differential_drivetrain;

import frc.robot.Utility.Accelerometer;
import frc.robot.Utility.Gyro;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
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

  double pastVel = 0;
  double maxAccel = 0;

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

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMaster.configSelectedFeedbackCoefficient(1);
    rightMaster.configSelectedFeedbackCoefficient(1);

    m_odometry.resetPosition(new Pose2d(), Gyro.getRotation2d());
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    m_feedForward = new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER);
  }
  public TalonSRX_Drivetrain(){
    this(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Gyro.getRotation2d(), ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * (double)leftMaster.getSelectedSensorPosition() / 4096.0 * Constants.ROBOT_WHEEL_CIRCUMFRENCE, ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * (double)rightMaster.getSelectedSensorPosition() / 4096.0 * Constants.ROBOT_WHEEL_CIRCUMFRENCE);


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

          var accel = wheelSpeeds.leftMetersPerSecond - pastVel;
          if(accel > maxAccel){
            System.out.println(accel);
            maxAccel = accel;
          }
            

          pastVel = wheelSpeeds.leftMetersPerSecond;
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
      -(double)leftMaster.getSelectedSensorVelocity() / 4096.0 * 10.0 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI,//the times 10 brings it from per 100ms to 1000ms
      -(double)rightMaster.getSelectedSensorVelocity() / 4096.0 * 10.0 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI
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
    return ((-(double)leftMaster.getSelectedSensorPosition(0) / 4096.0 * Constants.ROBOT_WHEEL_CIRCUMFRENCE) + 
      (-(double)rightMaster.getSelectedSensorPosition(0) / 4096.0 * Constants.ROBOT_WHEEL_CIRCUMFRENCE)) / 2.0;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.differential_drivetrain;

import frc.robot.Constants;
import frc.robot.Utility.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class CANSparkMax_Drivetrain extends Drivetrain {

  private CANSparkMax leftMaster, rightMaster; //These are the motor controllers that act as masters for the drivetrain
  private CANPIDController leftPID, rightPID;  //These allow access to inbuilt closed loops in the motor controllers
  private CANEncoder leftEncoder, rightEncoder;//These allow the encoders to be read
  private CANSparkMax[] leftSlaves, rightSlaves;


  /**
   * Creates a new CANSparkMax_Drivetrain.
   */
  public CANSparkMax_Drivetrain(boolean isBrushless) {
    //Picks between brushed and brushless motors (ask the mechanical or electrical teams for that information)
    MotorType motorType = (isBrushless) ? MotorType.kBrushless : MotorType.kBrushed;

    leftMaster = new CANSparkMax(Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN, motorType);
    rightMaster = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN, motorType);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    leftPID = leftMaster.getPIDController();
    rightPID = rightMaster.getPIDController();

    leftSlaves = new CANSparkMax[Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX - 1];
    rightSlaves = new CANSparkMax[Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX - 1];

    int leftSlaveCount = 0;//this exists so that the correct address in the leftSlaves array is accessed.
    for(int i = Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_LEFT_MOTOR_IDS_MAX; ++i){
      leftSlaves[leftSlaveCount] = new CANSparkMax(i, motorType);//makes the slaves
      leftSlaves[leftSlaveCount].follow(leftMaster);//binds the slaves to the masters
      ++leftSlaveCount;
    }

    int rightSlaveCount = 0;//this exists so that the correct address in the rightSlaves array is accessed.
    for(int i = Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MIN + 1/*the +1 makes the slaves not override the master*/; i <= Constants.DRIVETRAIN_RIGHT_MOTOR_IDS_MAX; ++i){
      rightSlaves[rightSlaveCount] = new CANSparkMax(i, motorType);//makes the slaves
      rightSlaves[rightSlaveCount].follow(rightMaster);//binds the slaves to the masters
      ++rightSlaveCount;
    }

    leftMaster.setInverted(Constants.DRIVETRAIN_INVERT_FORWARD);
    rightMaster.setInverted(!Constants.DRIVETRAIN_INVERT_FORWARD);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    m_odometry.resetPosition(new Pose2d(), Gyro.getRotation2d());
    m_feedForward = new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Gyro.getRotation2d(), ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * leftEncoder.getPosition() * Constants.ROBOT_WHEEL_DIAMETER * Math.PI, ((Constants.DRIVETRAIN_INVERT_ENCODERS)? -1 : 1) * rightEncoder.getPosition() * Constants.ROBOT_WHEEL_DIAMETER * Math.PI);

    if(Constants.IS_SIMULATION_RUNNING){
      mySimulationPeriodic();
    }else{
      switch(controlMode){
        case PERCENT:
          leftMaster.set(leftTarget);
          rightMaster.set(rightTarget);
          break;
  
        case VELOCITY:
          var wheelSpeeds = getWheelSpeeds();
          leftMaster.setVoltage(m_feedForward.calculate(leftTarget, leftTarget - wheelSpeeds.rightMetersPerSecond));
          rightMaster.setVoltage(m_feedForward.calculate(rightTarget, rightTarget - wheelSpeeds.rightMetersPerSecond));
          break;

        case VOLTAGE:
          leftMaster.setVoltage(leftTarget);
          rightMaster.setVoltage(rightTarget);
          break;

        default:
          leftMaster.set(leftTarget);
          rightMaster.set(rightTarget);
          break;
      }
    }
  }


  @Override
  public double[] getLeftOutputs() {
    double[] output = new double[leftSlaves.length + 1];

    output[0] = leftMaster.get();
    for(int i = 1; i < output.length; ++i){
      output[i] = leftSlaves[i].get();
    }

    return output;
  }

  @Override
  public double[] getRightOutputs() {
    double[] output = new double[rightSlaves.length + 1];

    output[0] = rightMaster.get();
    for(int i = 1; i < output.length; ++i){
      output[i] = rightSlaves[i].get();
    }

    return output;
  }


  //Ramsete code
  //Everything is in meters
  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity() / 60 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI,//rpm to rps to m/s
      rightEncoder.getVelocity() / 60 * Constants.ROBOT_WHEEL_DIAMETER * Math.PI
    );
  }

  @Override
  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  //In meters
  @Override
  public double getAverageEncoderDistance() {
    return ((leftEncoder.getPosition() * Constants.ROBOT_WHEEL_DIAMETER * Math.PI) + 
      (rightEncoder.getPosition() * Constants.ROBOT_WHEEL_DIAMETER * Math.PI)) / 2;
  }
}

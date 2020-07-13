/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.manipulator_mover;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath.*;

public class PWM_ManipulatorMoverSegment extends ManipulatorMoverSegment {
  /**
   * Creates a new PWM_ManipulatorMoverSegment.
   */
  private int[] motorIDs;
  private PWM[] motors;
  private Encoder encoder;
  private double countsPerRevolution = 360;
  private boolean[] motorInversion;//stores whether or not each motor is inverted
  /**
   * 
   * @param _length
   * @param _axis
   * @param _minAngle
   * @param _maxAngle
   * @param _motorIDs
   * @param _motorInversion
   * @param encoderChannels
   */
  public PWM_ManipulatorMoverSegment(double _length, Vector3 _axis, double _minAngle, double _maxAngle, int[] _motorIDs, boolean[] _motorInversion, int[] encoderChannels) {
    super(_length, _axis, _minAngle, _maxAngle);
    motorIDs = _motorIDs;
    motors = new PWM[motorIDs.length];
    for(int i = 0; i < motorIDs.length; ++i){
      motors[i] = new PWM(motorIDs[i]);
    }
    encoder = new Encoder(encoderChannels[0], encoderChannels[1]);
    motorInversion = _motorInversion;
  }
  /**
   * 
   * @param PWMMax the pulse width that corresponds to the maximum value
   * @param PWMDeadbandMax the deadband going up
   * @param PWMCenter the pulse width that corresponds to the off value
   * @param PWMDeadbandMin the deadband going down
   * @param PWMMin the pulse width that corresponds to the minimum value
   */
  public void configPWM(double PWMMax, double PWMDeadbandMax, double PWMCenter, double PWMDeadbandMin, double PWMMin){
    for(PWM motor : motors){
      motor.setBounds(PWMMax, PWMDeadbandMax, PWMCenter, PWMDeadbandMin, PWMMin);
    }
  }

  @Override
  protected void updateEncoder(){
    encoderAngle = (double)encoder.get() / countsPerRevolution * 360;
  }

  @Override
  protected void updateMotor(){
    for(int i = 0; i < motors.length; ++i){
      if(motorInversion[i]){
        motors[i].setSpeed(-motorOutput);
      }else{
        motors[i].setSpeed(motorOutput);
      }
    }
  }
}

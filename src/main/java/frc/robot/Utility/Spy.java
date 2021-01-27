/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// figure out what this is supposed to do
// I think this is supposed to observe the state of the robot
// The code was not made for this functionality
// this would be a retrofit


package frc.robot.Utility;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spy extends SubsystemBase {
  
  static private Spy spyInstance = new Spy();

  private Spy() {

  }

  static public Spy getSpy(){
    return spyInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

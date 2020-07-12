/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

import frc.robot.ExtraMath.*;
import frc.robot.Utility.FileOutput;
import frc.robot.commands.manipulator_mover.GoToPosition;
import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.subsystems.manipulator_mover.ManipulatorMoverSegment;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /*
  //for unity testing v1
  ManipulatorMoverSegment segment = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment2 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment segment3 = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment4 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment5 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);
  //ManipulatorMoverSegment segment6 = new ManipulatorMoverSegment(0.5, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment7 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);
  //ManipulatorMoverSegment segment8 = new ManipulatorMoverSegment(0.5, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment9 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);

  

  void inverse(double x, double y, double z){
    inverse(new Vector3(x, y, z));
  }

  void inverse(Vector3 target){
    //segment9.inverseKinematics(target);
    segment4.inverseKinematics(target, new Vector3(0, 0, 0));
    segment.realign();
    segment2.realign();
    segment3.realign();
    segment4.realign();
    //segment5.realign();
    //segment6.realign();
    //segment7.realign();
    //segment8.realign();
    //segment9.realign();
    segment.forwardKinematics();
    makeFrame();
  }

  void inverse(Vector3 target, Vector3 anchor){
    //segment9.inverseKinematics(target)
    segment4.inverseKinematics(target, anchor);
    segment.realign();
    segment2.realign();
    segment3.realign();
    segment4.realign();
    //segment5.realign();
    //segment6.realign();
    //segment7.realign();
    //segment8.realign();
    //segment9.realign();
    segment.forwardKinematics();
    makeFrame();
  }

  void lerpInverse(Vector3 start, Vector3 end, int steps){
    for(int i = 0; i < steps; ++i){
      inverse(Vector3.lerp(start, end, (double)i / (double)steps));
    }
  }

  void slerpInverse(Vector3 start, Vector3 end, int steps){
    for(int i = 0; i < steps; ++i){
      inverse(Vector3.slerp(start, end, (double)i / (double)steps));
    }
  }

  void slerpInverse(Vector3 start, Vector3 end, Vector3 anchorStart, Vector3 anchorEnd, int steps){
    for(int i = 0; i < steps; ++i){
      inverse(
        Vector3.slerp(start, end, (double)i / (double)steps),
        Vector3.lerp(anchorStart, anchorEnd, (double)i / (double)steps)
         );
    }
  }

  */

  /*
  //for unity testing v2
  ManipulatorMoverSegment segment = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment2 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment segment3 = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment4 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment[] segments = {segment, segment2, segment3, segment4};
  ManipulatorMover manipulatorMover = new ManipulatorMover(segments);

  void slerpInverse(Vector3 start, Vector3 end, int steps){
    for(int i = 0; i < steps; ++i){
      manipulatorMover.setTarget(Vector3.slerp(start, end, (double)i / (double)steps));
      manipulatorMover.updateKinematics();
    }
  }

  void slerpInverse(Vector3 start, Vector3 end, Vector3 anchorStart, Vector3 anchorEnd, int steps){
    for(int i = 0; i < steps; ++i){
      manipulatorMover.setTarget(Vector3.slerp(start, end, (double)i / (double)steps));
      manipulatorMover.setAnchor(Vector3.lerp(anchorStart, anchorEnd, (double)i / (double)steps));
      manipulatorMover.updateKinematics();
    }
  }
  */

  /*
  //for unity testing v4
  void slerpInverse(ManipulatorMover manipulatorMover, Vector3 start, Vector3 end, Vector3 subStart, Vector3 subEnd, int steps){
    for(int i = 0; i < steps; ++i){
      manipulatorMover.setTarget(Vector3.slerp(start, end, (double)i / (double)steps));
      manipulatorMover.setSubTarget(0, Vector3.slerp(subStart, subEnd, (double)i / (double)steps));
      manipulatorMover.updateKinematics();
    }
  }

  void slerpInverse(ManipulatorMover manipulatorMover, Vector3 start, Vector3 end, int steps){
    for(int i = 0; i < steps; ++i){
      manipulatorMover.setTarget(Vector3.slerp(start, end, (double)i / (double)steps));
      manipulatorMover.updateKinematics();
    }
  }
  */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    FileOutput.clearFile("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt");
    m_robotContainer = new RobotContainer();
    /*
    ManipulatorMover manipulatorMover = m_robotContainer.manipulatorMover;
    manipulatorMover.enableSubTarget(0);
    slerpInverse(manipulatorMover, new Vector3(0, 2, 0), new Vector3(0, 1, 1), new Vector3(0, 1, 0), new Vector3(-0.5, 0.5, 0), 100);
    slerpInverse(manipulatorMover, new Vector3(0, 1, 1), new Vector3(-1, 1, -1), new Vector3(-0.5, 0.5, 0), new Vector3(-0.5, 0.5, 0), 100);
    slerpInverse(manipulatorMover, new Vector3(-1, 1, -1), new Vector3(-1, 1, -1), new Vector3(-0.5, 0.5, 0), new Vector3(0, 0.5, 0), 100);
    slerpInverse(manipulatorMover, new Vector3(-1, 1, -1), new Vector3(-1, 1, -1), new Vector3(0, 0.5, 0), new Vector3(0.5, 0.5, 0), 100);
    slerpInverse(manipulatorMover, new Vector3(-1, 1, -1), new Vector3(1, 1, -1), new Vector3(0.5, 0.5, 0), new Vector3(0.5, 0.5, 0), 100);
    slerpInverse(manipulatorMover, new Vector3(1, 1, -1), new Vector3(1, 1, 0), new Vector3(0.5, 0.5, 0), new Vector3(, 0.5, 0), 100);
    */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.path2.schedule();
    //m_robotContainer.path.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /*
    Vector3 target = new Vector3(RobotContainer.controller.getX(Hand.kLeft), 1, -RobotContainer.controller.getY(Hand.kLeft));
    target = target.normalized().mult(2);
    m_robotContainer.goToPosition = new GoToPosition(m_robotContainer.manipulatorMover, target, 10);
    m_robotContainer.goToPosition.schedule();
    */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

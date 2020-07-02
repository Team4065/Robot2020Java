/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

import frc.robot.ExtraMath.*;
import frc.robot.subsystems.manipulator_mover.ManipulatorMoverSegment;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//for unity testing
import java.io.*;
//


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  ManipulatorMoverSegment segment = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment2 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment segment3 = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment4 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment5 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);
  //ManipulatorMoverSegment segment6 = new ManipulatorMoverSegment(0.5, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment7 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);
  //ManipulatorMoverSegment segment8 = new ManipulatorMoverSegment(0.5, Vector3.right, -90, 90);
  //ManipulatorMoverSegment segment9 = new ManipulatorMoverSegment(0.5, Vector3.forward, -90, 90);

  //for unity testing
  void makeFrame(){
    try (FileWriter outFile = new FileWriter("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt", true);
        BufferedWriter bWriter = new BufferedWriter(outFile);
        PrintWriter out = new PrintWriter(bWriter)) {

      out.println();
      out.println(segment.end_worldspace.toCSV());
      out.println(segment2.end_worldspace.toCSV());
      out.println(segment3.end_worldspace.toCSV());
      out.println(segment4.end_worldspace.toCSV());
      //out.println(segment5.end_worldspace.toCSV());
      //out.println(segment6.end_worldspace.toCSV());
      //out.println(segment7.end_worldspace.toCSV());
      //out.println(segment8.end_worldspace.toCSV());
      //out.println(segment9.end_worldspace.toCSV());
      

    } catch (IOException e) {
    e.printStackTrace();
    }
  }

  void inverse(double x, double y, double z){
    inverse(new Vector3(x, y, z));
  }

  void inverse(Vector3 target){
    //segment9.inverseKinematics(target);
    segment4.inverseKinematics(target);
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

  //

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    segment.setChildSegment(segment2);
    segment2.setChildSegment(segment3);
    segment3.setChildSegment(segment4);
    //segment4.setChildSegment(segment5);
    //segment5.setChildSegment(segment6);
    //segment6.setChildSegment(segment7);
    //segment7.setChildSegment(segment8);
    //segment8.setChildSegment(segment9);

    //clears the file
    try (FileWriter outFile = new FileWriter("C:\\Users\\colli\\Desktop\\InverseKinematicsOutput.txt")) {

    } catch (IOException e) {
      e.printStackTrace();
    }

    segment.forwardKinematics();
    makeFrame();

    slerpInverse(new Vector3(0, 2, 0), new Vector3(1, 0.5, 1), 100);
    slerpInverse(new Vector3(1, 0.5, 1), new Vector3(1, 1, -1), 100);
    slerpInverse(new Vector3(1, 1, -1), new Vector3(-1, 1, 1), 100);
    slerpInverse(new Vector3(-1, 1, 1), new Vector3(-2, 1, 0), 100);
    slerpInverse(new Vector3(-2, 1, 0), new Vector3(-1, 1, -1), 100);
    slerpInverse(new Vector3(-1, 1, -1), new Vector3(0, 1, -2), 100);
    slerpInverse(new Vector3(0, 1, -2), new Vector3(1, 1, -1), 100);
    slerpInverse(new Vector3(1, 1, -1), new Vector3(1, 1.5, 1), 100);
    //slerpInverse(new Vector3(0, 5, 0), new Vector3(3, 3, 0), 100);
    //slerpInverse(new Vector3(3, 3, 0), new Vector3(3, 0, 0), 100);
    //slerpInverse(new Vector3(3, 0, 0), new Vector3(3, 0, 3), 100);
    //slerpInverse(new Vector3(3, 0, 3), new Vector3(1, 3, 1), 100);
    //slerpInverse(new Vector3(1, 3, 1), new Vector3(1, 1, 1), 100);
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
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

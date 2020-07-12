/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath.*;

import frc.robot.subsystems.differential_drivetrain.TalonSRX_Drivetrain;
import frc.robot.subsystems.differential_drivetrain.CANSparkMax_Drivetrain;

import frc.robot.commands.differential_drivetrain.TankDrive;
import frc.robot.commands.differential_drivetrain.ArcadeDrive;

import frc.robot.subsystems.manipulator_mover.*;
import frc.robot.commands.manipulator_mover.*;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static XboxController controller = new XboxController(RobotMap.CONTROLS_MAIN_CONTROLLER_ID);

  //TalonSRX_Drivetrain drivetrain = new TalonSRX_Drivetrain(false, false);
  CANSparkMax_Drivetrain drivetrain = new CANSparkMax_Drivetrain(false, true);

  TankDrive tankdrive = new TankDrive(drivetrain);
  ArcadeDrive arcadedrive = new ArcadeDrive(drivetrain);

  //for unity testing v3
  ManipulatorMoverSegment segment = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment2 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment segment3 = new ManipulatorMoverSegment(2/4.0, Vector3.forward, -90, 90);
  ManipulatorMoverSegment segment4 = new ManipulatorMoverSegment(2/4.0, Vector3.right, -90, 90);
  ManipulatorMoverSegment[] segments = {segment, segment2, segment3, segment4};
  ManipulatorMover manipulatorMover = new ManipulatorMover(segments);

  //GoToPosition goToPosition = new GoToPosition(manipulatorMover, new Vector3(1, 1, 1), 1);
  Path2 path2 = new Path2(manipulatorMover);
  Path path = new Path(manipulatorMover);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadedrive);
    //manipulatorMover.setDefaultCommand(goToPosition);
    //manipulatorMover.setDefaultCommand(path);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

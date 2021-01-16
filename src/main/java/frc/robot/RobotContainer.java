/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Utility.Vector3;
import frc.robot.Utility.PathLoader;
import frc.robot.Utility.RamseteCommandBuilder;
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

  

  TalonSRX_Drivetrain drivetrain = new TalonSRX_Drivetrain(false);
  //CANSparkMax_Drivetrain drivetrain = new CANSparkMax_Drivetrain(true);

  TankDrive tankdrive = new TankDrive(drivetrain);
  ArcadeDrive arcadedrive = new ArcadeDrive(drivetrain);
  
  //ManipulatorMover manipulatorMover = new ManipulatorMover();

  //RamseteCommand drivetrainPath = new RamseteCommandBuilder(drivetrain, new PathLoader("Somewhere over the rainbow.")).getCommand();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadedrive);

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

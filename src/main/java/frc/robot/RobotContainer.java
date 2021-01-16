// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utility.PathLoader;
import frc.robot.Utility.RamseteCommandBuilder;
import frc.robot.commands.differential_drivetrain.ArcadeDrive;
import frc.robot.commands.differential_drivetrain.ArcadeDriveTracking;
import frc.robot.subsystems.differential_drivetrain.TalonFX_Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TalonFX_Drivetrain m_drivetrain = new TalonFX_Drivetrain();

  private final JoystickButton m_TrackTape = new JoystickButton(Constants.mainController, 0);

  private int autoChoice = 0;
  private final PathLoader m_leftAutoPath = new PathLoader("LeftAuto.JSON");
  private final RamseteCommand m_leftAuto = (new RamseteCommandBuilder(m_drivetrain, m_leftAutoPath)).getCommand();

  private final PathLoader m_rightAutoPath = new PathLoader("RightAuto.JSON");
  private final RamseteCommand m_rightAuto = (new RamseteCommandBuilder(m_drivetrain, m_rightAutoPath)).getCommand();

  private final PathLoader m_trenchAutoPath = new PathLoader("TrenchAuto.JSON");
  private final RamseteCommand m_trenchAuto = (new RamseteCommandBuilder(m_drivetrain, m_trenchAutoPath)).getCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_TrackTape.whileHeld(new ArcadeDriveTracking(m_drivetrain), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    switch(autoChoice){
      case 0:
        return m_leftAuto;
      case 1:
        return m_rightAuto;
      case 2:
        return m_trenchAuto;
      default:
        return new SequentialCommandGroup();//do nothing
    }
    
  }
}

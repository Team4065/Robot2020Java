/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility.Motor;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CharacterizeRotation;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain2;
import frc.robot.subsystems.TalonSRX_DifferentialDrivetrain;





/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Joystick m_controller = new Joystick(0);

  //DifferentialDrivetrain m_drivetrain = new TalonSRX_DifferentialDrivetrain(
  //  0.154, 
  //  new WPI_TalonSRX(1), new WPI_TalonSRX(4),
  //  new WPI_TalonSRX[]{new WPI_TalonSRX(2)}, new WPI_TalonSRX[]{new WPI_TalonSRX(5)}
  //  );

  DifferentialDrivetrain2 m_drivetrain = new DifferentialDrivetrain2(
    0.154,
    new Motor(1,"TalonSRX"), new Motor(4, "TalonSRX"),
    new Motor[]{new Motor(2, "TalonSRX")}, new Motor[]{new Motor(5, "TalonSRX")}
    );

  public RobotContainer() {
    m_drivetrain.configFeedforward(0.991, 3.27724, 1.04051);
    m_drivetrain.configRotationFeedForward(0.0455661717576, 0.00660299524632, 0.0000578739909339);
    //m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_controller, 2, 180));
    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_controller));
    configureButtonBindings();//0.145
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
    return new ExampleCommand();//new CharacterizeRotation(m_drivetrain);
  }
}

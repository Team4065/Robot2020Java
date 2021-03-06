/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Utility.Gyro;
import frc.robot.Utility.Limelight;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.MotorType;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Drivetrain.TrackObject.WhatToTrack;
import frc.robot.subsystems.DifferentialDrivetrain;





import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

//mport edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Joystick m_controller = new Joystick(0);
  JoystickButton turn = new JoystickButton(m_controller, 1);

  //DifferentialDrivetrain m_drivetrain = new TalonSRX_DifferentialDrivetrain(
  //  0.154, 
  //  new WPI_TalonSRX(1), new WPI_TalonSRX(4),
  //  new WPI_TalonSRX[]{new WPI_TalonSRX(2)}, new WPI_TalonSRX[]{new WPI_TalonSRX(5)}
  //  );

  DifferentialDrivetrain m_drivetrain = new DifferentialDrivetrain(
    0.154,
    new Motor(1, MotorType.TalonSRX), new Motor(4, MotorType.TalonSRX),
    new Motor[]{new Motor(2, MotorType.TalonSRX)}, new Motor[]{new Motor(5, MotorType.TalonSRX)}
    );

  // The robot's subsystems and commands are defined here...
  //Subsystems
  Shooter m_shooter;
  Kicker m_kicker;
  //Commands
  Shoot m_shoot;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Gyro.reset();
    Gyro.calibrate();
    Limelight.setPipeline(0);
    m_drivetrain.configFeedforward(0.991, 3.27724, 1.04051);
    m_drivetrain.configRotationFeedForward(0.0910536461972, 0.00629517415169, 0.0000576713721188);
    m_drivetrain.configFeedforwardSided(
      1.00760991292, 3.37832138739, -0.0405742345827,
      1.00702736431, 3.25734393828, -0.0234916065671
     );
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_controller, 2, 270));
    //m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_controller));
    configureButtonBindings();//0.145
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    turn.whileHeld(new TrackObject(m_drivetrain, WhatToTrack.Ball));
    //turn.whenPressed(new ToRotation(m_drivetrain));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //An ExampleCommand will run in autonomous
    return m_shoot;
  }
}

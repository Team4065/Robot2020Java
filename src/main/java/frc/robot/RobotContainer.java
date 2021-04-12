/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Flywheel.FlywheelToSpeed;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.IntakeSpit;
import frc.robot.commands.Intake.IntakeSuck;
import frc.robot.commands.Lift.LiftDeploy;
import frc.robot.commands.Lift.LiftDown;
import frc.robot.commands.Lift.LiftRetract;
import frc.robot.commands.Lift.LiftToHeight;
import frc.robot.commands.Lift.LiftUp;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Utility.Motor;
import frc.robot.Utility.Motor.MotorType;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Compressor m_compressor = new Compressor();
  Joystick m_controller = new Joystick(0);
  Joystick m_buttonbox = new Joystick(1);

  // The robot's subsystems and commands are defined here...
  final DifferentialDrivetrain m_drivetrain = new DifferentialDrivetrain(
    0.159/*wheel diameter*/,
    1 / 9.54, 
    false,
    false,
    new Motor(Constants.LEFT_DRIVETRAIN_MASTER, MotorType.CANSparkMax),
    new Motor(Constants.RIGHT_DRIVETRAIN_MASTER, MotorType.CANSparkMax),
    new Motor[]{new Motor(Constants.LEFT_DRIVTRAIN_SLAVES[0], MotorType.CANSparkMax)},
    new Motor[]{new Motor(Constants.RIGHT_DRIVETRAIN_SLAVES[0], MotorType.CANSparkMax)}
  );

  final Lift m_lift = new Lift();
  final Intake m_intake = new Intake();
  final Flywheel m_flywheel = new Flywheel();
  final Feeder m_feeder = new Feeder();
  final Conveyor m_conveyor = new Conveyor();

  private final ExampleCommand m_autoCommand = new ExampleCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.configFeedforwardSided(
      0.5319812402757178, 2.83098038352017, 0.06669021995516602,
      0.5148996120213958, 2.8125908259961605, 0.07363291020991329);
    m_drivetrain.enableBrakeMode(true);
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_controller, 1, Math.PI / 2));
    //m_intake.setDefaultCommand(new IntakeDeploy(m_intake));//so it deploys on start
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
    
    new JoystickButton(m_buttonbox, 3/* button label */).whileHeld(new LiftUp(m_lift));
    new JoystickButton(m_buttonbox, 4/* button label */).whileHeld(new LiftDown(m_lift));
    new JoystickButton(m_buttonbox, 1/* button label */).whenPressed(new LiftDeploy(m_lift));
    new JoystickButton(m_buttonbox, 2/* button label */).whenPressed(new LiftRetract(m_lift));
    new JoystickButton(m_buttonbox, 5).whenPressed(new LiftToHeight(m_lift, 1));
    new JoystickButton(m_buttonbox, 6).whenPressed(new LiftToHeight(m_lift, 0));

    new JoystickButton(m_buttonbox, 7).whileHeld(new Shoot(m_flywheel, m_feeder, m_conveyor, 80)).whenReleased(new FlywheelToSpeed(m_flywheel, 0));
    //new JoystickButton(m_buttonbox, 7).whenPressed(new PrepareAmmo(m_feeder));
    //new JoystickButton(m_buttonbox, 8).whenPressed(new FeedAmmo(m_feeder));
    new JoystickButton(m_controller, 1).whenPressed(new IntakeDeploy(m_intake));
    new JoystickButton(m_controller, 2).whenPressed(new IntakeRetract(m_intake));
    new JoystickButton(m_controller, 3).whileHeld(new IntakeSuck(m_intake));
    new JoystickButton(m_controller, 4).whileHeld(new IntakeSpit(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_drivetrain.findFeedForwardGainsLeft;
    //return new ExampleCommand();
  }
}

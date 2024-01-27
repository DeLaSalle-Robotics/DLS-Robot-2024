package frc.robot;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.FalconShooterMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Define subsystems and commands
  private final FalconShooterMotorSubsystem m_shooterMotor = new FalconShooterMotorSubsystem();
  private final IntakeSubsystem m_IntakeMotor = new IntakeSubsystem();
  //private final SwerveSubsystem m_swerve = new SwerveSubsystem();

  // setting up Xbox controller
  private final XboxController m_joystick = new XboxController(0);
  private Trigger controller_A = new JoystickButton(m_joystick, 1);
  private Trigger controller_B = new JoystickButton(m_joystick, 2);
  private Trigger controller_X = new JoystickButton(m_joystick, 3);
  private Trigger controller_Y = new JoystickButton(m_joystick, 4);


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    /*m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                () -> -m_joystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_joystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));*/

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*
    onTrue schedules the command when the button is pressed.
    whileTrue schedules the command when the button is pressed, and cancels the command when the button is released.
    toggleTrue toggles the command on every press: schedules if not currently scheduled, and cancels if scheduled.
    */
    /*controller_A.toggleOnTrue(new Shooter(() -> 0.5, m_shooterMotor));
    controller_B.toggleOnTrue(new Intake(() -> 0.5, m_IntakeMotor));
    controller_X.onTrue(new SwerveZeroHeading(m_swerve));*/

    controller_A.whileTrue(new Shooter(() -> 0.25, m_shooterMotor));
    controller_B.whileTrue(new Shooter(() -> 0.5, m_shooterMotor));
    controller_X.whileTrue(new Shooter(() -> 0.75, m_shooterMotor));
    controller_Y.whileTrue(new Shooter(() -> 1.0, m_shooterMotor));

  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

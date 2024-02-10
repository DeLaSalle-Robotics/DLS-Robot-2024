package frc.robot;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.FalconShooterMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NEOShooterMotorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.NEOShooter;
import frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.POVButton; // Use this for D-pad button mapping
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Define subsystems and commands
  private final FalconShooterMotorSubsystem m_shooterMotor = new FalconShooterMotorSubsystem();
  private final IntakeSubsystem m_IntakeMotor = new IntakeSubsystem();
  private final SwerveSubsystem m_swerve = new SwerveSubsystem();
  private final NEOShooterMotorSubsystem m_NeoShooterMotorSubsystem = new NEOShooterMotorSubsystem();

  // Declare Xbox Controller
  private final XboxController m_joystick = new XboxController(OIConstants.kDriverControllerPort);

  // Button declarations
  private Trigger controller_A = new JoystickButton(m_joystick, 1);
  private Trigger controller_B = new JoystickButton(m_joystick, 2);
  private Trigger controller_X = new JoystickButton(m_joystick, 3);
  private Trigger controller_Y = new JoystickButton(m_joystick, 4);

  // Unused buttons - these will work fine if you simply uncomment them
  // private Trigger controller_LB = new JoystickButton(m_joystick, 5);
  // private Trigger controller_RB = new JoystickButton(m_joystick, 6);
  // private Trigger controller_Share = new JoystickButton(m_joystick, 7);
  // private Trigger controller_Menu = new JoystickButton(m_joystick, 8);
  // private Trigger controller_LStick = new JoystickButton(m_joystick, 9); // Left stick click
  // private Trigger controller_RStick = new JoystickButton(m_joystick, 10); // Right stick click

  // D-pad buttons - these will work fine if you simply uncomment them
  // private Trigger controller_N = new POVButton(m_joystick, 0); // D-pad up
  // private Trigger controller_NE = new POVButton(m_joystick, 45); // D-pad up-right
  // private Trigger controller_E = new POVButton(m_joystick, 90); // D-pad right
  // private Trigger controller_SE = new POVButton(m_joystick, 135); // D-pad down-right
  // private Trigger controller_S = new POVButton(m_joystick, 180); // D-pad down
  // private Trigger controller_SW = new POVButton(m_joystick, 225); // D-pad down-left
  // private Trigger controller_W = new POVButton(m_joystick, 270); // D-pad left
  // private Trigger controller_NW = new POVButton(m_joystick, 315); // D-pad up-left

  /* Additional notes:
    The joysticks (as well as LT and RT, since they have analog input) are linked to a different axis ID, listed below
    0 - Left Stick X
    1 - Left Stick Y*
    2 - Left Trigger (LT)**
    3 - Right Trigger (RT)**
    4 - Right Stick X
    5 - Right Stick Y*

    These can each be retrieved by writing:
    m_joystick.getRawAxis(axis);

    * For y-axis, the value increases when moving the stick down, and decreases when moving the stick up
    ** This value cannot be below zero
  */


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                () -> -m_joystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_joystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*
    onTrue schedules the command when the button is pressed.
    whileTrue schedules the command when the button is pressed, and cancels the command when the button is released.
    toggleTrue toggles the command on every press: schedules if not currently scheduled, and cancels if scheduled.
    */
    controller_A.toggleOnTrue(new Shooter(() -> 0.5, m_shooterMotor));
    controller_B.toggleOnTrue(new Intake(() -> 0.5, m_IntakeMotor));
    controller_X.onTrue(new SwerveZeroHeading(m_swerve));
    controller_Y.onTrue(new NEOShooter(() -> SmartDashboard.getNumber("Speed",0) , m_NeoShooterMotorSubsystem));
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

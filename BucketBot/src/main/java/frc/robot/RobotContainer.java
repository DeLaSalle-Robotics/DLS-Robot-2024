package frc.robot;

import frc.robot.subsystems.FalconShooterMotorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.Shooter;
import frc.robot.commands.climber.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Robot;

public class RobotContainer {

  // Define subsystems and commands
  private final FalconShooterMotorSubsystem m_shooterMotor = new FalconShooterMotorSubsystem();
  //private final IntakeSubsystem m_IntakeMotor = new IntakeSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  //private final SwerveSubsystem m_swerve = new SwerveSubsystem();

  // setting up Xbox controller
  private final XboxController m_joystick = new XboxController(0);
   //private Trigger controller_A = new JoystickButton(m_joystick, 1);
  // private Trigger controller_B = new JoystickButton(m_joystick, 2);
  private Trigger controller_X = new JoystickButton(m_joystick, 3);
  private Trigger controller_Y = new JoystickButton(m_joystick, 4);

  private Trigger controller_dpad_N = new POVButton(m_joystick, 0);
  private Trigger controller_dpad_S = new POVButton(m_joystick, 180);


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    /*m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                () -> -m_joystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_joystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_joystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));*/
    //m_shooterMotor.setDefaultCommand(new Shooter(() -> m_joystick.getLeftY(), () -> m_joystick.getRightY(), m_shooterMotor));
    // m_shooterMotor.setDefaultCommand(new Shooter(() -> MathUtil.applyDeadband(m_joystick.getLeftY(), 0.1), () -> MathUtil.applyDeadband(m_joystick.getRightY(), 0.1), m_shooterMotor));
    
    m_ClimberSubsystem.setDefaultCommand(new ClimberTest(
      () -> MathUtil.applyDeadband(m_joystick.getLeftY() * -0.75, 0.1),
      m_ClimberSubsystem));
      
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*
    onTrue schedules the command when the button is pressed.
    whileTrue schedules the command when the button is pressed, and cancels the command when the button is released.
    toggleTrue toggles the command on every press: schedules if not currently scheduled, and cancels if scheduled.
    */

    System.out.println("configured bindings");
    // Tele-op bindings
    if(RobotState.isTeleop()){

      // Move the climber motors up or down
      controller_dpad_N.whileTrue(new ClimberManual(true, m_ClimberSubsystem));
      controller_dpad_S.whileTrue(new ClimberManual(false, m_ClimberSubsystem));

      

    // Test mode bindings
    } else if(RobotState.isTest()){
      
      // Move the climber motors up or down
      controller_dpad_N.whileTrue(new ClimberTest(() -> 0.2, m_ClimberSubsystem));
      controller_dpad_S.whileTrue(new ClimberTest(() -> -0.35, m_ClimberSubsystem));
      
      // Swap which climber motor is active
      controller_Y.onTrue(new ClimberSwap(m_ClimberSubsystem));

      controller_X.whileTrue(new Shooter(
        () -> SmartDashboard.getNumber("Shooter Speed", 0.0), 
        () -> SmartDashboard.getNumber("Shooter Speed", 0.0),
        m_shooterMotor
      ));
  
    }

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

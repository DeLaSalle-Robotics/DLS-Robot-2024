package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.FalconShooterMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.Shooter;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {

  // Make sure to define swerve subsystem BEFORE vision subsystem
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VisionSubsystem m_vision = new VisionSubsystem(m_swerve);

  // Define subsystems and commands
  private final FalconShooterMotorSubsystem m_shooterMotor = new FalconShooterMotorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // setting up Xbox controller
  private final XboxController m_joystick = new XboxController(0);
  private Trigger controller_A = new JoystickButton(m_joystick, 1);
  private Trigger controller_B = new JoystickButton(m_joystick, 2);
  private Trigger controller_X = new JoystickButton(m_joystick, 3);


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_swerve,
    //   () -> MathUtil.applyDeadband(m_joystick.getLeftY(),
    //       OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_joystick.getLeftX(),
    //       OperatorConstants.LEFT_X_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_joystick.getRightX(),
    //       OperatorConstants.RIGHT_X_DEADBAND),
    //   m_joystick::getYButtonPressed,
    //   m_joystick::getAButtonPressed,
    //   m_joystick::getXButtonPressed,
    //   m_joystick::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = m_swerve.driveCommand(
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_joystick.getRightX(),
    //     () -> m_joystick.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation 
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_joystick.getRightX());

    // Command driveFieldOrientedDirectAngleSim = m_swerve.simDriveCommand(
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_joystick.getRightX());

    m_swerve.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);

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
    controller_B.toggleOnTrue(new Intake(() -> 0.5, m_intake));
    controller_X.onTrue(Commands.run(() -> m_swerve.zeroGyro()));

    new JoystickButton(m_joystick, 1).onTrue((new InstantCommand(m_swerve::zeroGyro)));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_swerve.getAutonomousCommand(Constants.Auton.PathFileName, true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerve.setMotorBrake(brake);
  }

}

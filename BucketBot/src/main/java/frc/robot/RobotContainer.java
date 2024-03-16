package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.FalconShooterMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.Rumble;
import frc.robot.commands.WatchTarget;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  // Define subsystems
  // Don't change the order that these are declared in, since some subsystems require others to function
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final FalconShooterMotorSubsystem m_shooterMotor = new FalconShooterMotorSubsystem();
  private final ControllerSubsystem m_controller = new ControllerSubsystem();

  // These subsystems require other subsystems and MUST be declared after all others
  private final VisionSubsystem m_vision = new VisionSubsystem(m_swerve);
  private final IntakeSubsystem m_intake = new IntakeSubsystem(m_controller);



  // Setting up Xbox controller
  private final XboxController m_joystick = m_controller.getController();
  
  // Controller buttons:

  private Trigger controller_A = new JoystickButton(m_joystick, 1);
  private Trigger controller_B = new JoystickButton(m_joystick, 2);
  private Trigger controller_X = new JoystickButton(m_joystick, 3);
  private Trigger controller_Y = new JoystickButton(m_joystick, 4);

  private Trigger controller_LB = new JoystickButton(m_joystick, 5);
  private Trigger controller_RB = new JoystickButton(m_joystick, 6);

  private Trigger controller_Share = new JoystickButton(m_joystick, 7);
  private Trigger controller_Menu = new JoystickButton(m_joystick, 8);

  private Trigger controller_LStick = new JoystickButton(m_joystick, 9);
  private Trigger controller_RStick = new JoystickButton(m_joystick, 10);

  private Trigger controller_dpad_N = new POVButton(m_joystick, 0);
  private Trigger controller_dpad_NE = new POVButton(m_joystick, 45);
  private Trigger controller_dpad_E = new POVButton(m_joystick, 90);
  private Trigger controller_dpad_SE = new POVButton(m_joystick, 135);
  private Trigger controller_dpad_S = new POVButton(m_joystick, 180);
  private Trigger controller_dpad_SW = new POVButton(m_joystick, 225);
  private Trigger controller_dpad_W = new POVButton(m_joystick, 270);
  private Trigger controller_dpad_NW = new POVButton(m_joystick, 315);


  Command driveFieldOrientedWatchTarget;




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
    Command driveFieldOrientedAngularVelocity = m_swerve.driveCommand(
        () -> -MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(m_joystick.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> m_joystick.getLeftTriggerAxis());

    driveFieldOrientedWatchTarget = m_swerve.driveAutoAimCommand(
      () -> -MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      m_vision.getPhotonCamera()
    );

    // Command driveFieldOrientedDirectAngleSim = m_swerve.simDriveCommand(
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_joystick.getRightX());

    m_swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);

    SmartDashboard.putNumber("Intake Target Speed", 0.5);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*
    onTrue schedules the command when the button is pressed.
    whileTrue schedules the command when the button is pressed, and cancels the command when the button is released.
    toggleOnTrue toggles the command on every press: schedules if not currently scheduled, and cancels if scheduled.
    */

    // Final bindings, plz don't delete or comment!!!
    
    // Spin intake
    // controller_A.whileTrue(new Intake(() -> SmartDashboard.getNumber("Intake Target Speed", 0.5), m_intake));
    // controller_B.whileTrue(new Intake(() -> -SmartDashboard.getNumber("Intake Target Speed", 0.5), m_intake));

    controller_A.whileTrue(new Intake(() -> SmartDashboard.getNumber("Intake Target Speed", 0.5), m_intake));
    controller_B.whileTrue(new Intake(() -> -SmartDashboard.getNumber("Intake Target Speed", 0.5), m_intake));

    
    // Zero heading
    controller_RStick.onTrue((new InstantCommand(m_swerve::zeroGyro)));

    // controller_Y -> onTrue -> Fire shooter
    // controller_LB -> whileTrue -> Auto aim heading
    // controller_Share -> onTrue -> Enter winch mode
    
    // controller_dpad_N -> whileTrue -> Move climber up
    // controller_dpad_S -> whileTrue -> Move climber down
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
    // m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerve.setMotorBrake(brake);
  }

}

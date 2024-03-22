package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
import edu.wpi.first.wpilibj.Joystick;
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
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ControllerSubsystem m_ControllerSubsystem = new ControllerSubsystem();

  // These subsystems require other subsystems and MUST be declared after all others
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_SwerveSubsystem);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(m_ControllerSubsystem);



  // Setting up Xbox controller and flight joysticks
  private final XboxController m_controller = m_ControllerSubsystem.getController();
  private final Joystick m_flightJoystick = m_ControllerSubsystem.getFlightJoystick(true);
  
  // Xbox Controller buttons:

  private Trigger controller_A = new JoystickButton(m_controller, 1);
  private Trigger controller_B = new JoystickButton(m_controller, 2);
  private Trigger controller_X = new JoystickButton(m_controller, 3);
  private Trigger controller_Y = new JoystickButton(m_controller, 4);

  private Trigger controller_LB = new JoystickButton(m_controller, 5);
  private Trigger controller_RB = new JoystickButton(m_controller, 6);

  private Trigger controller_Share = new JoystickButton(m_controller, 7);
  private Trigger controller_Menu = new JoystickButton(m_controller, 8);

  private Trigger controller_LStick = new JoystickButton(m_controller, 9);
  private Trigger controller_RStick = new JoystickButton(m_controller, 10);

  private Trigger controller_dpad_N = new POVButton(m_controller, 0);
  private Trigger controller_dpad_NE = new POVButton(m_controller, 45);
  private Trigger controller_dpad_E = new POVButton(m_controller, 90);
  private Trigger controller_dpad_SE = new POVButton(m_controller, 135);
  private Trigger controller_dpad_S = new POVButton(m_controller, 180);
  private Trigger controller_dpad_SW = new POVButton(m_controller, 225);
  private Trigger controller_dpad_W = new POVButton(m_controller, 270);
  private Trigger controller_dpad_NW = new POVButton(m_controller, 315);


  // Joystick buttons are pretty easy, just follow the format below
  // Change "1" to whichever numbered button you wish to use.
  private Trigger joystick_1 = new JoystickButton(m_flightJoystick, 1);


  Command driveFieldOrientedAngularVelocity;
  Command driveFieldOrientedWatchTarget;




  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation 
    // right stick controls the angular velocity of the robot
    driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kLeftYDeadband),
        () -> -MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kLeftXDeadband),
        () -> -MathUtil.applyDeadband(m_controller.getRightX(), OperatorConstants.kRightXDeadband),
        () -> m_controller.getLeftTriggerAxis());

    driveFieldOrientedWatchTarget = m_SwerveSubsystem.driveAutoAimCommand(
      () -> -MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kLeftYDeadband),
      () -> -MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kLeftXDeadband),
      m_VisionSubsystem.getPhotonCamera()
    );

    m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

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
    controller_A.whileTrue(new Intake(m_IntakeSubsystem, () -> SmartDashboard.getNumber("Intake Target Speed", 0.5)));
    controller_B.whileTrue(new Intake(m_IntakeSubsystem, () -> -SmartDashboard.getNumber("Intake Target Speed", 0.5)));

    
    // Zero heading
    controller_RStick.onTrue((new InstantCommand(m_SwerveSubsystem::zeroGyro)));

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
    return m_SwerveSubsystem.getAutonomousCommand(Constants.AutoConstants.kPathFileName, true);
  }

  public void setDriveMode()
  {
    // m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  public void setMotorBrake(boolean brake)
  {
    m_SwerveSubsystem.setMotorBrake(brake);
  }

}






// Old drive commands
// These are no longer used but I refuse to get rid of them until competition

// AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_SwerveSubsystem,
    //   () -> MathUtil.applyDeadband(m_controller.getLeftY(),
    //       OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_controller.getLeftX(),
    //       OperatorConstants.LEFT_X_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_controller.getRightX(),
    //       OperatorConstants.RIGHT_X_DEADBAND),
    //   m_controller::getYButtonPressed,
    //   m_controller::getAButtonPressed,
    //   m_controller::getXButtonPressed,
    //   m_controller::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = m_SwerveSubsystem.driveCommand(
    //     () -> MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_controller.getRightX(),
    //     () -> m_controller.getRightY());

    // Command driveFieldOrientedDirectAngleSim = m_SwerveSubsystem.simDriveCommand(
    //     () -> MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_controller.getRightX());

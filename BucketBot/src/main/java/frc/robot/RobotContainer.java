package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.Shooter;
import frc.robot.commands.ShooterAnalog;
import frc.robot.commands.LED;
import frc.robot.commands.climber.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  // Define subsystems
  // Don't change the order that these are declared in, since some subsystems require others to function
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ControllerSubsystem m_ControllerSubsystem = new ControllerSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  // These subsystems require other subsystems and MUST be declared after all others
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(m_ControllerSubsystem);
  // private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_SwerveSubsystem);

  // Allows picking autonomous routines from SmartDashboard
  private final SendableChooser<Command> m_autoChooser;

  // Setting up controllers and flight joysticks
  private final XboxController m_controller = m_ControllerSubsystem.getDriveController();
  private final Joystick m_flightJoystick = m_ControllerSubsystem.getClimbController();
  private final GenericHID m_testController = m_ControllerSubsystem.getTestController();
  

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

  // Other triggers
  private Trigger controller_RT = new Trigger(() -> m_controller.getRightTriggerAxis() > 0.1);


  // Flight joystick buttons are pretty easy, just follow the format below
  // Change "1" to whichever numbered button you wish to use.
  private Trigger joystick_1 = new JoystickButton(m_flightJoystick, 1);
  private Trigger joystick_2 = new JoystickButton(m_flightJoystick, 2);
  private Trigger joystick_3 = new JoystickButton(m_flightJoystick, 3);
  private Trigger joystick_4 = new JoystickButton(m_flightJoystick, 4);
  private Trigger joystick_5 = new JoystickButton(m_flightJoystick, 5);
  private Trigger joystick_6 = new JoystickButton(m_flightJoystick, 6);
  private Trigger joystick_7 = new JoystickButton(m_flightJoystick, 7);
  private Trigger joystick_8 = new JoystickButton(m_flightJoystick, 8);
  private Trigger joystick_9 = new JoystickButton(m_flightJoystick, 9);
  private Trigger joystick_10 = new JoystickButton(m_flightJoystick, 10);
  private Trigger joystick_11 = new JoystickButton(m_flightJoystick, 11);


  // Test controller buttons
  // A is skipped because it doesn't work well
  private Trigger testController_B = new JoystickButton(m_testController, 2);
  private Trigger testController_X = new JoystickButton(m_testController, 3);
  private Trigger testController_Y = new JoystickButton(m_testController, 4);
  private Trigger testController_LB = new JoystickButton(m_testController, 5);
  private Trigger testController_RB = new JoystickButton(m_testController, 6);
  private Trigger testController_Back = new JoystickButton(m_testController, 7);
  private Trigger testController_Start = new JoystickButton(m_testController, 8);
  private Trigger testController_LStick = new JoystickButton(m_testController, 9);
  private Trigger testController_RStick = new JoystickButton(m_testController, 10);

  





  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Register Named Commands
    NamedCommands.registerCommand("autoShooter", m_ShooterSubsystem.autoShooter(m_IntakeSubsystem));
    NamedCommands.registerCommand("autoIntake", m_IntakeSubsystem.autoIntake());

    // Build an auto chooser
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    SmartDashboard.putString("Robot State", "Have Note");
  }

  /**
   * Analog controller bindings for tele-op mode.
   */
  public void configureAnalogTeleop(){

    // Default drive command
    Command driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveCommand(
      () -> -MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kLeftYDeadband),
      () -> -MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kLeftXDeadband),
      () -> -MathUtil.applyDeadband(m_controller.getRightX(), OperatorConstants.kRightXDeadband),
      () -> m_controller.getLeftTriggerAxis()
    );
    
    // Set up default command for LED subsystem
    // Not bound to any controller action, just runs all the time
    m_LEDSubsystem.setDefaultCommand(new LED(m_LEDSubsystem, m_IntakeSubsystem));
    m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
    //m_ShooterSubsystem.setDefaultCommand(spinShooter);
  }


  /**
   * Digital controller bindings for tele-op mode.
   */
  public void configureBindingsTeleop() {
    /*
    onTrue schedules the command when the button is pressed.
    whileTrue schedules the command when the button is pressed, and cancels the command when the button is released.
    toggleOnTrue toggles the command on every press: schedules if not currently scheduled, and cancels if scheduled.
    */

    // Run intake forward
    controller_RT.whileTrue(
      new Intake(
        m_IntakeSubsystem, 
        () -> Constants.Intake.kIntakePower,
        () -> false
    ));

    // Run intake backward
    controller_RB.whileTrue(new Intake(
      m_IntakeSubsystem, 
      () -> -Constants.Intake.kIntakeReversePower,
      () -> true
    ));

    // Clear hasNote status after feeding to shooter
    joystick_1.onFalse(new InstantCommand(() -> m_IntakeSubsystem.setHasNote(false)));

    // Zero heading
    controller_Menu.onTrue(new InstantCommand(m_SwerveSubsystem::zeroGyro));


    // Feed intake to shooter
    joystick_1.whileTrue(new Intake(
      m_IntakeSubsystem, 
      () -> SmartDashboard.getNumber("Intake Feeder Speed", Constants.Intake.kIntakeFeederPower), 
      () -> true
    ));

    // Spin shooter forward
    joystick_3.whileTrue(new Shooter(m_ShooterSubsystem));
    
    // Spin shooter backward
    joystick_8.whileTrue(new ShooterAnalog(
      m_ShooterSubsystem, 
      () -> -Constants.Shooter.kShooterReversePower
    ));

    // Run intake in reverse
    joystick_2.whileTrue(new Intake(
      m_IntakeSubsystem, 
      () -> -SmartDashboard.getNumber("Intake Reverse Speed", Constants.Intake.kIntakeReversePower), 
      () -> true
    ));
  

    // Move extender up
    joystick_6.whileTrue(new ClimberExtenderSimple(
      m_ClimberSubsystem, 
      () -> Constants.Climber.kExtenderExtendPower
    ));

    // Move extender down
    joystick_7.whileTrue(new ClimberExtenderSimple(
      m_ClimberSubsystem, 
      () -> Constants.Climber.kExtenderRetractPower
    ));

    // Move winch up
    /*joystick_11.whileTrue(new ClimberWinchSimple(
      m_ClimberSubsystem, 
      () -> Constants.Climber.kWinchExtendPower
    ));*/

    // Move winch down
    /* joystick_10.whileTrue(new ClimberWinchSimple(
      m_ClimberSubsystem, 
      () -> Constants.Climber.kWinchRetractPower
    ));*/
  }


  /**
   * Analog controller bindings for test mode.
   */
  public void configureAnalogTest(){

    // Controls the climber motors
    Command climbTestMode = new ClimberTest(
      m_ClimberSubsystem,
      () -> -MathUtil.applyDeadband(m_testController.getRawAxis(5), Constants.OperatorConstants.kRightYDeadband) * 0.2, // RightY
      () -> -MathUtil.applyDeadband(m_testController.getRawAxis(1), Constants.OperatorConstants.kLeftYDeadband) * 0.3 // LeftY
    );

    // Spin shooter motor at variable power
    Command spinShooterTestMode = new ShooterAnalog(
      m_ShooterSubsystem,
      () -> MathUtil.applyDeadband(m_testController.getRawAxis(3), Constants.OperatorConstants.kTriggerDeadband)
    );

    // Run intake at variable power
    Command spinIntakeTestMode = new Intake(
      m_IntakeSubsystem, 
      () -> MathUtil.applyDeadband(m_testController.getRawAxis(2), Constants.OperatorConstants.kTriggerDeadband), 
      () -> false
    );


    m_ClimberSubsystem.setDefaultCommand(climbTestMode);
    m_ShooterSubsystem.setDefaultCommand(spinShooterTestMode);
    m_IntakeSubsystem.setDefaultCommand(spinIntakeTestMode);

    // Cancel swerve's command if it still exists from switching modes
    CommandScheduler.getInstance().unregisterSubsystem(m_SwerveSubsystem);
  }


  /**
   * Digital controller bindings for test mode.
   */
  public void configureBindingsTest(){
    
    // Fire shooter
    testController_Y.whileTrue(new Intake(
      m_IntakeSubsystem, 
      () -> SmartDashboard.getNumber("Intake Feeder Speed", Constants.Intake.kIntakeFeederPower), 
      () -> true
    ));

    // Reverse intake
    testController_LB.whileTrue(new Intake(
      m_IntakeSubsystem,
      () -> -SmartDashboard.getNumber("Intake Reverse Speed", Constants.Intake.kIntakeReversePower),
      () -> false
    ));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    //return m_ShooterSubsystem.autoShooter(m_IntakeSubsystem);
  }

  public void setMotorBrake(boolean brake)
  {
    m_SwerveSubsystem.setMotorBrake(brake);
  }


  public void resetWinch(){
    m_ClimberSubsystem.setWinchUpperLimit();
  }

  public void initHasNote(){
    m_IntakeSubsystem.setHasNote(true);
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

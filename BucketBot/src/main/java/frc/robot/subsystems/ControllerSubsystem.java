package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ControllerSubsystem extends SubsystemBase {

  private final XboxController m_joystick;

  /**
   * Currently only used for global rumble functionality, though it MAY be possible to move all bindings here to keep
   * {@link RobotContainer} cleaner.
   */
  public ControllerSubsystem() {
    super();
    m_joystick = new XboxController(Constants.OIConstants.kDriverControllerPort);
  }

  /**
   * Returns the current Xbox controller used for driving. 
   * @return The current controller.
   */
  public XboxController getController(){
    return m_joystick;
  }


  /**
   * Rumble the controller.
   * <p><i>You have to set the rumble back to 0 manually or else the controller won't stop rumbling.</i>
   * @param motor The rumble motor to use, can be right, left, or both.
   * @param intensity Intensity of the rumble, from 0 to 1.
   */
  public void rumble(RumbleType motor, double intensity){
    m_joystick.setRumble(motor, intensity);
  }




  // Default subsystem methods


  // Unused
  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  // Unused
  // Query some boolean state, such as a digital sensor.
  public boolean exampleCondition() {
    return false;
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {

  }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ControllerSubsystem extends SubsystemBase {

  private final XboxController m_controller;
  private final Joystick m_flightJoystickR;
  private final Joystick m_flightJoystickL;
  private final GenericHID m_testController;

  /**
   * Currently only used for global rumble functionality.
   */
  public ControllerSubsystem() {
    super();
    m_controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    m_flightJoystickR = new Joystick(Constants.OperatorConstants.kFlightJoystickPortR);
    m_flightJoystickL = new Joystick(Constants.OperatorConstants.kFlightJoystickPortL);
    m_testController = new GenericHID(Constants.OperatorConstants.kTestControllerPort);
  }

  /**
   * Returns the current Xbox controller used for driving. 
   * @return The controller in port 0.
   */
  public XboxController getController(){
    return m_controller;
  }


  /**
   * Returns one of the flight joysticks.
   * @param right True returns joystick R, false returns joystick L
   * @return The selected flight joystick
   */
  public Joystick getFlightJoystick(boolean right){
    if (right){
      return m_flightJoystickR;
    } else {
      return m_flightJoystickL;
    }
  }

  
  /**
   * Returns the PS4 Controller used for test mode.
   * @return The controller in port 3.
   */
  public GenericHID getTestController(){
    return m_testController;
  }




  /**
   * Rumble the controller.
   * <p><i>You have to set the rumble back to 0 manually or else the controller won't stop rumbling.</i>
   * @param motor The rumble motor to use, can be right, left, or both.
   * @param intensity Intensity of the rumble, from 0 to 1.
   */
  public void rumble(RumbleType motor, double intensity){
    m_controller.setRumble(motor, intensity);
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

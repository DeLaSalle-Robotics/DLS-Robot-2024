package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDSubsystem extends SubsystemBase {

  // Declare Falcon controllers
  private final m_leds;
  
  // LEDSubsystem constructor
  public LEDSubsystem() {
    super();
    m_leds = new LEDController(Constants.LED.Pin);
  }

  /**
   * Set the LED to a certain color
   * @param power Double between -1.0 and 1.0
   */
  public void set(Colors color){
    m_leds.set(color);
  }

   /**
   * Turn off the LED
   */
   public void off(){
      m_leds.off();
   }

  // Called once per scheduler run
  @Override
  public void periodic() {
  }


  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
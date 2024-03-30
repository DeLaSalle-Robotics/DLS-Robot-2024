package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LEDSubsystem extends SubsystemBase {

    // Must be a PWM header, not MXP or DIO
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

  // LEDSubsystem constructor
  public LEDSubsystem() {
    super();
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(Constants.LED.kLEDPWM);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.kNumberLEDs);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * Set the LED to a certain color
   * @param color Color to set the LED to
   */
  public void set(Color color){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setLED(i, color);
     }
     m_led.setData(m_ledBuffer);
  }

   /**
   * Turn off the LED
   */
   public void off(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setLED(i, Color.kBlack);
     }
     m_led.setData(m_ledBuffer);
   }

  // Called once per scheduler run
  @Override
  public void periodic() {
  }


  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
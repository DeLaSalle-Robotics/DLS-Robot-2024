package frc.robot.subsystems;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LEDSubsystem extends SubsystemBase {

    // Must be a PWM header, not MXP or DIO
    private final AddressableLED m_led;
    private final AddressableLEDSim m_led_sim;
    private final AddressableLEDBuffer m_ledBuffer;
    // NetworkTable BooleanSubscribers
    BooleanSubscriber InZoneSub;
    BooleanSubscriber OnTargetSub;
    BooleanSubscriber NoteSub;
    BooleanSubscriber RestingSub;

    private int m_rainbowFirstPixelHue;
    
    public enum LED_State {
      WAITING,
      NO_NOTE,
      HAVE_NOTE,
      IN_ZONE,
      ON_TARGET
    }
    private LED_State currentState;
  // LEDSubsystem constructor
  public LEDSubsystem() {
    super();
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(Constants.LED.kLEDPWM);
    m_led_sim = new AddressableLEDSim(m_led);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.kNumberLEDs);
    m_led.setLength(m_ledBuffer.getLength());
    m_led_sim.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    currentState = LED_State.WAITING;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    InZoneSub = table.getBooleanTopic("InZone").subscribe(false);
    NoteSub = table.getBooleanTopic("Note").subscribe(false);
    OnTargetSub = table.getBooleanTopic("OnTarget").subscribe(false);
    RestingSub = table.getBooleanTopic("Resting").subscribe(false);
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
        // Sets the specified LED to the HSV values for black
        m_ledBuffer.setLED(i, Color.kBlack);
     }
     m_led.setData(m_ledBuffer);
   }

   public void rainbow(){
    for (var i=0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + ( i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 200);
    }
    m_rainbowFirstPixelHue += 1;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
   }

public Command ledRed(Color kRed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          this.set(kRed);
        });
  }




  // Called once per scheduler run
  @Override
  public void periodic() {
    //Reaching out to other subsystems to identify the state for LED display
    if (NoteSub.getAsBoolean()) {this.currentState = LED_State.HAVE_NOTE;}else {currentState = LED_State.NO_NOTE;}
    if (InZoneSub.getAsBoolean() && NoteSub.getAsBoolean()
        ) {this.currentState = LED_State.IN_ZONE;}
    if (OnTargetSub.getAsBoolean() && NoteSub.getAsBoolean()
        ) {this.currentState = LED_State.ON_TARGET;}
    if (RestingSub.getAsBoolean()) {this.currentState = LED_State.WAITING;}
// Key is to have a series of methods that define the states within subsystems
    switch (currentState) {
        case HAVE_NOTE -> this.set(Color.kBlue);
        case NO_NOTE -> { this.off();}
        case IN_ZONE-> this.set(Color.kRed);
        case WAITING-> this.rainbow();
        case ON_TARGET -> this.set(Color.kGreen);
        default -> this.off();
      }
    SmartDashboard.putString("Led State", currentState.toString());
  }


  public void disabledPeriodic() {
    this.rainbow();
  }

  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
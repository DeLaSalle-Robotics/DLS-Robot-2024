package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public class LedSubsystem extends SubsystemBase {

  private static AddressableLED m_led = new AddressableLED(0);
  private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  public LedSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }


  public void setSection(int start, int end, int red, int green, int blue){
    for(int i = start; i < end; i++){
      m_ledBuffer.setRGB(i, red, green, blue);
    }
  }


  // Default subsystem methods


  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {}


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}

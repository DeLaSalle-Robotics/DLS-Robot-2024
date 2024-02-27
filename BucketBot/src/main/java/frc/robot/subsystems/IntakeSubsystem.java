package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

  // Declare a SparkMax as the intake motor controller
  //private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);

  /**
  Deprecated as channel is not properly set yet, just a warning
   */
  //private final DigitalInput m_LimitSwitch = new DigitalInput(Constants.Intake.kIntakeLimitSwitchID);

  // IntakeSubsystem constructor
  public IntakeSubsystem() {
    super();
  }

  /**
   * Retrieves the state of the limit switch
   * @return whether the limit switch is pressed
   */
  public boolean testLimitSwitch() {
    return false;
    //return m_LimitSwitch.get();
  }


  /**
   * Sets the intake motor to the given speed
   * @param speed Speed of the intake motor, between -1.0 and 1.0
   */
  public void spin(double speed) {
    //m_IntakeMotor.set(speed);
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
  public void periodic() {}


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}

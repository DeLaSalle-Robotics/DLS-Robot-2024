package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Rumble;


public class IntakeSubsystem extends SubsystemBase {

  // Declare a SparkMax as the intake motor controller
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);
  private final Encoder m_IntakeEncoder = new Encoder(0, 1);

  private final ControllerSubsystem m_controller;
  private final Trigger m_feedback;
  
  public IntakeSubsystem(ControllerSubsystem controllerSubsystem) {
    super();
    m_controller = controllerSubsystem;

    m_feedback = new Trigger(() -> this.noteDetected());
    m_feedback.onTrue(new Rumble(m_controller, () -> 1.0, true));
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
    m_IntakeMotor.set(speed);
  }


  public double getEncoderRate(){
    return m_IntakeEncoder.getRate();
  }


  public boolean noteDetected(){
    return (this.getEncoderRate() <= -300);
  }


  /**
   * Used to send rumble feedback when the note is automatically stopped.
   */
  public ControllerSubsystem getControllerSubsystem(){
    return m_controller;
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
    SmartDashboard.putNumber("Encoder Rate", m_IntakeEncoder.getRate());
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}

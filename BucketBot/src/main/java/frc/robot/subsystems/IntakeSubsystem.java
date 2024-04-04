package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.Rumble;
import frc.robot.commands.Shooter;


public class IntakeSubsystem extends SubsystemBase {

  // Intake motor controller
  private final CANSparkMax m_IntakeMotor;
  private final Encoder m_IntakeEncoder;

  // Controller subsystem and rumble feedback trigger
  private final ControllerSubsystem m_ControllerSubsystem;
  private final Trigger m_feedback;

  // Median filter to avoid accidentally randomly stopping the intake
  private MedianFilter encoderFilter = new MedianFilter(10);

  // Tells us whether or not the robot has a note in its intake
  private boolean m_hasNote = false;
  
  
  public IntakeSubsystem(ControllerSubsystem controllerSubsystem) {
    super();
    m_ControllerSubsystem = controllerSubsystem;

    // Intake motor controller and encoder
    // The encoder is NOT the same as the motor's encoder!
    m_IntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);
    m_IntakeEncoder = new Encoder(Constants.Intake.kIntakeEncoderDIO1, Constants.Intake.kIntakeEncoderDIO2);

    // Rumble feedback
    m_feedback = new Trigger(() -> this.noteDetected());
    m_feedback.onTrue(new Rumble(m_ControllerSubsystem, () -> 0.5, true));
  }


  /**
   * Set the intake motor to a raw speed, from -1.0 to 1.0.
   * @param speed
   */
  public void spinDirect(double speed){
    m_IntakeMotor.set(speed);
  }


  /**
   * Stop the intake motor.
   */
  public void stopIntake() {
    m_IntakeMotor.set(0.0);
  }


  /**
   * Gets the rate of the encoder.
   * <p>This is NOT the motor encoder.
   * @return The rate of the encoder in "distance per second."
   */
  public double getEncoderRate(){
    return encoderFilter.calculate(m_IntakeEncoder.getRate());
  }


  /**
   * Returns whether or not a note is currently being moved forward through the intake.
   * @return True if a note was detected, false otherwise.
   */
  public boolean noteDetected(){
    return (this.getEncoderRate() <= -300);
  }


  /**
   * Sets whether or not the robot has a note in its intake.
   * @param hasNote The value to set to.
   */
  public void setHasNote(boolean hasNote){
    m_hasNote = hasNote;
    if (hasNote){
      SmartDashboard.putString("Note Status", "Has Note");
    } else {
      SmartDashboard.putString("Note Status", "Empty");
    }
  }


  /**
   * Checks whether or not the robot has a note in its intake.
   * @return True if there is a note in the intake, false otherwise.
   */
  public boolean hasNote(){
    return m_hasNote;
  }

  public Command autoIntake(IntakeSubsystem intake){
    return Commands.sequence(
      new Intake(this, () -> Constants.Intake.kIntakePower, () -> false).withTimeout(5.0),
      new Intake(this, () -> -Constants.Intake.kIntakeReversePower, () -> true).withTimeout(0.25),
      new Intake(this, () -> Constants.Intake.kIntakeReversePower, () -> true).withTimeout(0.25),
      new Intake(this, () -> -Constants.Intake.kIntakeReversePower, () -> true).withTimeout(0.25),
      new Intake(this, () -> Constants.Intake.kIntakeReversePower, () -> true).withTimeout(0.25)
    );
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
    SmartDashboard.putNumber("Motor Velocity", m_IntakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Temp", m_IntakeMotor.getMotorTemperature());
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}

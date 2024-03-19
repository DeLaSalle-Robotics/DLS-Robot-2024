package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;
import frc.robot.commands.Rumble;


public class IntakeSubsystem extends SubsystemBase {

  // Intake motor controller
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);

  // NOT linked to the intake motor, this is for stopping the note within the intake
  private final Encoder m_IntakeEncoder = new Encoder(0, 1);

  // Controller subsystem and rumble feedback trigger
  private final ControllerSubsystem m_controller;
  private final Trigger m_feedback;

  // PID Controller for the intake motor
  private final PIDController m_intakeController;
  private MedianFilter encoderFilter = new MedianFilter(10);
  
  
  public IntakeSubsystem(ControllerSubsystem controllerSubsystem) {
    super();
    m_controller = controllerSubsystem;

    // Intake PID controller
    SmartDashboard.putNumber("Intake kP", Constants.Intake.kPIntake);
    m_intakeController = new PIDController(SmartDashboard.getNumber("Intake kP", Constants.Intake.kPIntake), 0.0, 0.0);

    // Rumble feedback
    m_feedback = new Trigger(() -> this.noteDetected());
    m_feedback.onTrue(new Rumble(m_controller, () -> 1.0, true));
  }


  /**
   * Sets the intake motor to the given speed
   * <p><b>This is controlled by a PID!</b> Use {@link #spinDirect} to set raw speed, or {@link #stopIntake} to stop the motor.
   * <p><b>DO NOT use this to stop the intake motor!</b>
   * @param speed Speed of the intake motor, in rotations per minute
   */
  public void spin(double speed) {
    double rawSpeed = m_intakeController.calculate(m_IntakeMotor.getEncoder().getVelocity(), speed);
    m_IntakeMotor.set(MathUtil.clamp(rawSpeed, -0.4, 0.4));
    SmartDashboard.putNumber("Raw Motor Speed", rawSpeed);
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
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}

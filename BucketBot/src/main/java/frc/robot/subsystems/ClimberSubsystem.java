package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// neo pulls down
// falcon controls climbing


public class ClimberSubsystem extends SubsystemBase {
  
  // /** NEO that controls extension, controlled by a SparkMax. */
  // private final CANSparkMax m_extenderMotor = new CANSparkMax(Constants.Climber.kExtenderMotorID, MotorType.kBrushless);
  // /** TalonFX that controls climbing. */
  // private final TalonFX m_climberMotor = new TalonFX(Constants.Climber.kClimberMotorID);
  // private final DigitalInput m_limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitchID);

  private final PIDController m_extenderPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController m_climberPID = new PIDController(0.1, 0.0, 0.0);

  // ClimberSubsystem constructor
  public ClimberSubsystem() {
    super();

    /*

    SmartDashboard.putNumber("kPExtenderDown", Constants.Climber.kPExtenderDown);
    SmartDashboard.putNumber("kPClimberDown", Constants.Climber.kPClimberDown);
    SmartDashboard.putNumber("kPExtenderUp", Constants.Climber.kPExtenderUp);
    SmartDashboard.putNumber("kPClimberUp", Constants.Climber.kPClimberUp);

    // Invert climber motor
    m_climberMotor.setInverted(true);

    // Send the currently active motor to SmartDashboard for test mode
    SmartDashboard.putBoolean("Using Extender", true);

    // Reset the position of the climber motor encoder on startup when using Teleop
    if(RobotState.isTeleop()){
      m_climberMotor.setPosition(0.0);
    }

    */
  }


  /**
   * Spin the extender and climber motors at separate speeds.
   * @param eSpeed Speed of the extender motor
   * @param cSpeed Speed of the climber motor
   */
  public void spinMotors(double eSpeed, double cSpeed){
    
    /*

    // Set speeds of each motor
    m_extenderMotor.set(eSpeed);
    m_climberMotor.set(cSpeed);

    // Send the positions of the motors to SmartDashboard
    SmartDashboard.putNumber("Extender Motor", m_extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Motor", m_climberMotor.getPosition().getValueAsDouble());

    // Send the speeds of the motors to SmartDashboard
    SmartDashboard.putNumber("Extender Speed", eSpeed);
    SmartDashboard.putNumber("Climber Speed", cSpeed);

    */
  }


  /**
   * Spin both motors to a set position.
   * @param ePos Extender position, in centimeters.
   * @param cPos Climber position, in centimeters.
   */
  public void spinMotorsTo(double ePos, double cPos){

    /*

    m_extenderMotor.set(m_extenderPID.calculate(getExtenderPosition(), ePos));
    m_climberMotor.set(m_climberPID.calculate(getClimberPosition(), cPos));

    */
  }


  /**
   * Returns the position of the extender motor
   * @return Position of the extender motor in centimeters
   */
  public double getExtenderPosition(){
    return 0.0;
    // return m_extenderMotor.getEncoder().getPosition() * Constants.Climber.kExtenderCmPerRotation;
  }

  /**
   * Returns the position of the climber motor
   * @return Position of the climber motor in centimeters
   */
  public double getClimberPosition(){
    return 0.0;
    // return m_climberMotor.getPosition().getValueAsDouble() * Constants.Climber.kClimberCmPerRotation;
  }

  /**
   * Returns the velocity of the extender motor
   * @return Velocity of extender motor in RPM
   */
  public double getExtenderVelocity(){
    return 0.0;
    // return m_extenderMotor.getEncoder().getVelocity();
  }

  /**
   * Returns the velocity of the climber motor
   * @return Velocity of the climber motor in RPM
   */
  public double getClimberVelocity(){
    return 0.0;
    // return m_climberMotor.getVelocity().getValueAsDouble() * 60;
  }


  /**
   * Returns the state of the climber limit switch.
   * @return The state of the limit switch.
   */
  public boolean getSwitchState(){
    return false;
    // return m_limitSwitch.get();
  }


  /**
   * Swaps the currently active motor in test mode.
   * <p><b>This is only used in test mode.</b>
   */
  public void swapActiveMotor(){
    
    // Simple boolean swap code
    if(SmartDashboard.getBoolean("Using Extender", true)){
      SmartDashboard.putBoolean("Using Extender", false);
    } else {
      SmartDashboard.putBoolean("Using Extender", true);
    }
  }


  /**
   * Resets the extender motor encoder to zero.
   */
  public void resetExtenderEncoder(){
    // m_extenderMotor.getEncoder().setPosition(0.0);
  }


  /**
   * Resets the climb motor encoder to zero.
   */
  public void resetClimberEncoder(){
    // m_climberMotor.setPosition(0.0);
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
  public void periodic() {
    // SmartDashboard.putNumber("extenderVelocity", this.getExtenderVelocity());
    // SmartDashboard.putNumber("climberVelocity", this.getClimberVelocity());
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {

  }

}

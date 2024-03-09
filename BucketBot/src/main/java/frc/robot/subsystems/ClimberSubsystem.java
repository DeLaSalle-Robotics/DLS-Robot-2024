/*
 * Example subsystem
 * Copy this to create another subsystem
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// neo pulls down
// falcon controls climbing


public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax m_extenderMotor = new CANSparkMax(Constants.Climber.kExtenderMotorID, MotorType.kBrushless);
  private final TalonFX m_climberMotor = new TalonFX(Constants.Climber.kClimberMotorID);
  private final DigitalInput m_limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitchID);

  // ExampleSubsystem constructor
  public ClimberSubsystem() {
    super();
    SmartDashboard.putNumber("kPExtenderDown", Constants.Climber.kPExtenderDown);
    SmartDashboard.putNumber("kPClimberDown", Constants.Climber.kPClimberDown);
    SmartDashboard.putNumber("kPExtenderUp", Constants.Climber.kPExtenderUp);
    SmartDashboard.putNumber("kPClimberUp", Constants.Climber.kPClimberUp);

    // Set soft limits of the extender motor to not allow it to extend further until it reaches the limit switch
    m_extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_extenderMotor.getEncoder().setPosition(0.0);
    m_extenderMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f);
    m_extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, -Constants.Climber.kExtenderDistance);

    // Invert climber motor
    m_climberMotor.setInverted(true);
  }


  /**
   * Spin the extender and climber motors at separate speeds.
   * @param speed1 Speed of the extender motor
   * @param speed2 Speed of the climber motor
   */
  public void spinMotors(double speed1, double speed2){
    
    // Set speeds of each motor
    m_extenderMotor.set(speed1);
    m_climberMotor.set(speed2);

    // Send the positions of the motors to SmartDashboard
    SmartDashboard.putNumber("Extender Motor", m_extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Motor", m_climberMotor.getPosition().getValueAsDouble());

    // Send the speeds of the motors to SmartDashboard
    SmartDashboard.putNumber("Extender Speed", speed1);
    SmartDashboard.putNumber("Climber Speed", speed2);

    // Send the currently active motor to SmartDashboard for test mode
    SmartDashboard.putBoolean("Using Extender", true);
  }


  /**
   * Returns the position of the extender motor
   * @return Position of the extender motor in rotations
   */
  public double getExtenderPosition(){
    return m_extenderMotor.getEncoder().getPosition();
  }

  /**
   * Returns the position of the climber motor
   * @return Position of the climber motor in rotations
   */
  public double getClimberPosition(){
    return m_climberMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the velocity of the extender motor
   * @return Velocity of extender motor in RPM
   */
  public double getExtenderVelocity(){
    return m_extenderMotor.getEncoder().getVelocity();
  }

  /**
   * Returns the velocity of the climber motor
   * @return Velocity of the climber motor in RPM
   */
  public double getClimberVelocity(){
    return m_climberMotor.getVelocity().getValueAsDouble() * 60;
  }


  /**
   * Returns the state of the climber limit switch.
   * @return The state of the limit switch.
   */
  public boolean getSwitchState(){
    return m_limitSwitch.get();
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


  public void setSoftLimits(){
    m_extenderMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.kExtenderDistance);
    m_extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.0f);
    m_extenderMotor.getEncoder().setPosition(0.0);
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
    SmartDashboard.putNumber("extenderVelocity", this.getExtenderVelocity());
    SmartDashboard.putNumber("climberVelocity", this.getClimberVelocity());
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {

  }

}

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
  
  // Physical devices
  private final CANSparkMax m_extenderMotor;
  private final TalonFX m_winchMotor;
  private final DigitalInput m_limitSwitch;

  // PID controllers
  private final PIDController m_extenderPID;
  private final PIDController m_winchPID;



  // ClimberSubsystem constructor
  public ClimberSubsystem() {
    super();

    // Create physical devices
    m_extenderMotor = new CANSparkMax(Constants.Climber.kExtenderMotorID, MotorType.kBrushless);
    m_winchMotor = new TalonFX(Constants.Climber.kWinchMotorID);
    m_limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitchID);

    // Create PID controllers
    m_extenderPID = new PIDController(0.1, 0.0, 0.0);
    m_winchPID = new PIDController(0.1, 0.0, 0.0);

    // Create SmartDashboard kP values
    SmartDashboard.putNumber("kPExtenderDown", Constants.Climber.kPExtenderDown);
    SmartDashboard.putNumber("kPWinchDown", Constants.Climber.kPWinchDown);
    SmartDashboard.putNumber("kPExtenderUp", Constants.Climber.kPExtenderUp);
    SmartDashboard.putNumber("kPWinchUp", Constants.Climber.kPWinchUp);

    // Send the currently active motor to SmartDashboard for test mode
    SmartDashboard.putBoolean("Using Extender", true);

    // Invert winch motor
    m_winchMotor.setInverted(true);

    // Reset the position of the winch motor encoder on startup when using Teleop
    if(RobotState.isTeleop()){
      m_winchMotor.setPosition(0.0);
    }

  }


  /**
   * Spin the extender motor directly with no limits.
   * @param speed The speed to spin the extender motor at, from -1.0 to 1.0
   */
  public void spinExtender(double speed){
    m_extenderMotor.set(speed);
  }

  /**
   * Spin the extender motor directly with limits enabled.
   * @param speed The speed to spin the extender motor at, from -1.0 to 1.0
   * @param lowerLimit Lower limit of the extender position in centimeters
   * @param upperLimit Upper limit of the extender position in centimeters
   */
  public void spinExtender(double speed, double lowerLimit, double upperLimit){

    // Moving up and under the upper limit
    if(speed > 0 && this.getExtenderPosition() < upperLimit){
      m_extenderMotor.set(speed);

    // Moving down and above the lower limit
    } else if(speed < 0 && this.getExtenderPosition() > lowerLimit){
      m_extenderMotor.set(speed);

    // Speed is zero or a limit was passed
    } else {
      m_extenderMotor.set(0.0);
    }
  }



  /**
   * Spin the winch motor directly with no limits.
   * @param speed The speed to spin the winch motor at, from -1.0 to 1.0
   */
  public void spinWinch(double speed){
    m_winchMotor.set(speed);
  }


  /**
   * Spin the winch motor directly with limits enabled.
   * @param speed The speed to spin the winch motor at, from -1.0 to 1.0
   * @param lowerLimit Lower limit of the winch position in centimeters
   * @param upperLimit Upper limit of the winch position in centimeters
   */
  public void spinWinch(double speed, double lowerLimit, double upperLimit){

    // Moving up and under the upper limit
    if(speed > 0 && this.getWinchPosition() < upperLimit){
      m_winchMotor.set(speed);

    // Moving down and above the lower limit
    } else if(speed < 0 && this.getWinchPosition() > lowerLimit){
      m_winchMotor.set(speed);

    // Speed is zero or a limit was passed
    } else {
      m_winchMotor.set(0.0);
    }
  }


  /**
   * Spin both motors at a set velocity, using PID controllers.
   * @param ePos Extender position, in centimeters.
   * @param wPos Winch position, in centimeters.
   */
  public void spinMotorsAt(double eSpeed, double wSpeed){
    
    m_extenderMotor.set(m_extenderPID.calculate(getExtenderVelocity(), eSpeed));
    m_winchMotor.set(m_winchPID.calculate(getWinchVelocity(), wSpeed));

  }


  /**
   * Returns the position of the extender motor
   * @return Position of the extender motor in centimeters
   */
  public double getExtenderPosition(){
    return m_extenderMotor.getEncoder().getPosition() * Constants.Climber.kExtenderCmPerRotation;
  }

  /**
   * Returns the position of the winch motor
   * @return Position of the winch motor in centimeters
   */
  public double getWinchPosition(){
    return m_winchMotor.getPosition().getValueAsDouble() * Constants.Climber.kWinchCmPerRotation;
  }


  /**
   * Returns the velocity of the extender motor
   * @return Velocity of extender motor in cm/s
   */
  public double getExtenderVelocity(){
    
    // Raw velocity in rotations/min
    double rawVelocity = m_extenderMotor.getEncoder().getVelocity();

    // Return in cm/s
    return (rawVelocity / 60) * Constants.Climber.kExtenderCmPerRotation;
  }

  /**
   * Returns the velocity of the winch motor
   * @return Velocity of the winch motor in cm/s
   */
  public double getWinchVelocity(){

    // Raw velocity in rotations/s
    double rawVelocity = m_winchMotor.getVelocity().getValueAsDouble();

    // Return in cm/s
    return (rawVelocity * Constants.Climber.kWinchCmPerRotation);
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


  /**
   * Resets the extender motor encoder to zero.
   */
  public void resetExtenderEncoder(){
    m_extenderMotor.getEncoder().setPosition(0.0);
  }


  /**
   * Resets the winch motor encoder to zero.
   */
  public void resetWinchEncoder(){
    m_winchMotor.setPosition(0.0);
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

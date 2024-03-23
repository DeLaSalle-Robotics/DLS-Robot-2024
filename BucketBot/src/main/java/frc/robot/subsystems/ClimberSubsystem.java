package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  // Feedforward controllers
  private final SimpleMotorFeedforward m_extenderFF;
  private final SimpleMotorFeedforward m_winchFF;



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

    // Create Feedforward controllers
    m_extenderFF = new SimpleMotorFeedforward(Constants.Climber.kExtenderFFS, Constants.Climber.kExtenderFFV);
    m_winchFF = new SimpleMotorFeedforward(Constants.Climber.kWinchFFS, Constants.Climber.kWinchFFV);

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
   * Calculates the PID-Feedforward controller for the extender motor
   * @param speed Speed to spin the extender at, in cm/s
   * @return A clamped PID-Feedforward calculation
   */
  private double calculateExtenderPID(double speed){

    // Calculate feedforward and PID controllers and add them together
    double ffPower = m_extenderFF.calculate(speed);
    double pidPower = m_extenderPID.calculate(this.getExtenderVelocity(), speed);
    double power = ffPower + pidPower;

    // Clamp the above result and return
    return MathUtil.clamp(power, Constants.Climber.kExtenderMinPower, Constants.Climber.kExtenderMaxPower);
  }

  /**
   * Calculates the PID-Feedforward controller for the winch motor
   * @param speed Speed to spin the winch at, in cm/s
   * @return A clamped PID-Feedforward calculation
   */
  private double calculateWinchPID(double speed){

    // Calculate feedforward and PID controllers and add them together
    double ffPower = m_winchFF.calculate(speed);
    double pidPower = m_winchPID.calculate(this.getWinchVelocity(), speed);
    double power = ffPower + pidPower;

    // Clamp the above result and return
    return MathUtil.clamp(power, Constants.Climber.kWinchMinPower, Constants.Climber.kWinchMaxPower);
  }



  /**
   * Spin the extender motor directly with no limits.
   * @param power The speed to spin the extender motor at, from -1.0 to 1.0
   */
  public void spinExtender(double power){
    m_extenderMotor.set(power);
  }

  /**
   * Spin the extender motor directly with limits enabled.
   * @param power The speed to spin the extender motor at, from -1.0 to 1.0
   * @param lowerLimit Lower limit of the extender position in centimeters
   * @param upperLimit Upper limit of the extender position in centimeters
   */
  public void spinExtender(double power, double lowerLimit, double upperLimit){

    // Moving up and under the upper limit
    if(power > 0 && this.getExtenderPosition() < upperLimit){
      m_extenderMotor.set(power);

    // Moving down and above the lower limit
    } else if(power < 0 && this.getExtenderPosition() > lowerLimit){
      m_extenderMotor.set(power);

    // Speed is zero or a limit was passed
    } else {
      m_extenderMotor.set(0.0);
    }
  }



  /**
   * Spin the winch motor directly with no limits.
   * @param power The speed to spin the winch motor at, from -1.0 to 1.0
   */
  public void spinWinch(double power){
    m_winchMotor.set(power);
  }

  /**
   * Spin the winch motor directly with limits enabled.
   * @param power The speed to spin the winch motor at, from -1.0 to 1.0
   * @param lowerLimit Lower limit of the winch position in centimeters
   * @param upperLimit Upper limit of the winch position in centimeters
   */
  public void spinWinch(double power, double lowerLimit, double upperLimit){

    // Moving up and under the upper limit
    if(power > 0 && this.getWinchPosition() < upperLimit){
      m_winchMotor.set(power);

    // Moving down and above the lower limit
    } else if(power < 0 && this.getWinchPosition() > lowerLimit){
      m_winchMotor.set(power);

    // Speed is zero or a limit was passed
    } else {
      m_winchMotor.set(0.0);
    }
  }



  /**
   * Spin the extender at a given velocity with no limits.
   * @param speed Speed to spin the extender motor at, in cm/s.
   */
  public void spinExtenderAt(double speed){

    // Calculate feedforward and PID controllers
    double totalPower = this.calculateExtenderPID(speed);

    // Spin the extender with no limits
    this.spinExtender(totalPower);
  }

  /**
   * Spin the extender at a given velocity with limits enabled.
   * @param speed Speed to spin the extender motor at, in cm/s.
   * @param lowerLimit Lower limit of the extender position in centimeters.
   * @param upperLimit Upper limit of the extender position in centimeters.
   */
  public void spinExtenderAt(double speed, double lowerLimit, double upperLimit){

    // Calculate feedforward and PID controllers
    double totalPower = this.calculateExtenderPID(speed);

    // Spin the extender with limits enabled
    this.spinExtender(totalPower, lowerLimit, upperLimit);
  }



  /**
   * Spin the winch at a given velocity with no limits.
   * @param speed Speed to spin the winch motor at, in cm/s.
   */
  public void spinWinchAt(double speed){
    
    // Calculate feedforward and PID controllers
    double totalPower = this.calculateWinchPID(speed);

    // Spin the winch with no limits
    this.spinWinch(totalPower);
  }

  /**
   * Spin the winch at a given velocity.
   * @param speed Speed to spin the winch motor at, in cm/s.
   * @param lowerLimit Lower limit of the winch position in centimeters.
   * @param upperLimit Upper limit of the winch position in centimeters.
   */
  public void spinWinchAt(double speed, double lowerLimit, double upperLimit){

    // Calculate feedforward and PID controllers
    double totalPower = this.calculateWinchPID(speed);

    // Spin the winch with limits enabled
    this.spinWinch(totalPower, lowerLimit, upperLimit);
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
  public void setExtenderLowerLimit(){
    m_extenderMotor.getEncoder().setPosition(Constants.Climber.kExtenderEndpointDown);
  }


  /**
   * Resets the winch motor encoder to zero.
   */
  public void setWinchLowerLimit(){
    m_winchMotor.setPosition(Constants.Climber.kWinchEndpointDown);
  }


  /**
   * Gets the current corresponding to the torque output of the winch motor.
   * @return The absolute value of the torque current of the winch motor, between 0 and 327.68
   */
  public double getWinchCurrent(){
    return Math.abs(m_winchMotor.getTorqueCurrent().getValueAsDouble());
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
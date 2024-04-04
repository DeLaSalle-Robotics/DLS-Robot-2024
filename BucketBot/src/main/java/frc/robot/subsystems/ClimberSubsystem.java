package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Feedforward controllers
  private final ElevatorFeedforward m_extenderFF;

  // Velocity command for winch motor
  final VelocityVoltage m_velocityCmdWinch = new VelocityVoltage(0);

  // ClimberSubsystem constructor
  public ClimberSubsystem() {
    super();

    // Create physical devices
    m_extenderMotor = new CANSparkMax(Constants.Climber.kExtenderMotorID, MotorType.kBrushless);
    m_winchMotor = new TalonFX(Constants.Climber.kWinchMotorID);
    m_limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitchID);

    // reset motor configurations to avoid uncertain configurations
    m_extenderMotor.restoreFactoryDefaults();
    m_winchMotor.getConfigurator().apply(new TalonFXConfiguration());

    // Invert the winch motor
    m_winchMotor.setInverted(true);

    // Set brake mode
    m_winchMotor.setNeutralMode(NeutralModeValue.Brake);
    m_extenderMotor.setIdleMode(IdleMode.kBrake);

    // Create controller for winch motor
    // Slot 0 is a FF controller with gravity term and P controller
    // This is for manual mode where we have no load on the winch
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.Climber.kWinchFFS; // Voltage Required to move the motors at all. 
    slot0Configs.kV = Constants.Climber.kWinchFFV; // Same as kF - V per rot/s <-- Calculated from applied voltage/ spin velocity
    slot0Configs.kG = Constants.Climber.kWinchFFG;  // Holding power for constant gravitation loads
    slot0Configs.kP = Constants.Climber.kWinchKP;  // Proportional factor needs tuning
    slot0Configs.kI = Constants.Climber.kWinchKI;  // Integral factor useful if not getting to set point
    slot0Configs.kD = Constants.Climber.kWinchKD; // Derivative term for preventing overshoot
    m_winchMotor.getConfigurator().apply(slot0Configs, 0.050); //Put these control values into Slot0 in the talonFx controller- with a 50 ms overflow limit.
    
    // Create controllers for extender motor
    m_extenderFF = new ElevatorFeedforward(Constants.Climber.kExtenderFFS, 
                                            Constants.Climber.kExtenderFFV, 
                                            Constants.Climber.kExtenderFFG);
    m_extenderPID = new PIDController(Constants.Climber.kExtenderKP, 
                                      Constants.Climber.kExtenderKI,
                                      Constants.Climber.kExtenderKD);


    // Send the currently active motor to SmartDashboard for test mode
    SmartDashboard.putBoolean("Using Extender", true);
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
    return MathUtil.clamp(power, -1.0, 1.0);
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

    double setSpeed = speed;
    // Apply upper limit
    if(speed > 0 && getExtenderPosition() > Constants.Climber.kExtenderEndpointUp){
      setSpeed = 0;
    }
    // Apply lower limit
    else if(speed < 0 && getExtenderPosition() < Constants.Climber.kExtenderEndpointDown){
      setSpeed = 0;
    }

    // Calculate feedforward and PID controllers
    double totalPower = this.calculateExtenderPID(setSpeed);
    m_extenderMotor.set(totalPower);
  }



  /**
   * Spin the winch at a given velocity with no limits.
   * @param speed Speed to spin the winch motor at, in cm/s.
   */
  public void spinWinchAt(double speed){
    m_velocityCmdWinch.Slot = 0;
    double speedRPS = speed / Constants.Climber.kWinchCmPerRotation; // Convert from cm/s to rot/s
    m_winchMotor.setControl(m_velocityCmdWinch.withVelocity(speedRPS));
  }

  /**
   * Spin the winch at a given velocity.
   * @param speed Speed to spin the winch motor at, in cm/s.
   * @param lowerLimit Lower limit of the winch position in centimeters.
   * @param upperLimit Upper limit of the winch position in centimeters.
   */
  public void spinWinchAt(double speed, double lowerLimit, double upperLimit){
    
    double setSpeed = speed;
    // Apply upper limit
    if(speed > 0 && getWinchPosition() > Constants.Climber.kWinchEndpointUp){
      setSpeed = 0;
    }
    // Apply lower limit
    else if(speed < 0 && getWinchPosition() < Constants.Climber.kWinchEndpointDown){
      setSpeed = 0;
    }

    m_velocityCmdWinch.Slot = 0;
    double speedRPS = setSpeed / Constants.Climber.kWinchCmPerRotation; // Convert from cm/s to rot/s
    m_winchMotor.setControl(m_velocityCmdWinch.withVelocity(speedRPS));
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
   * Resets the winch motor encoder to its default value.
   */
  public void setWinchUpperLimit(){
    m_winchMotor.setPosition(Constants.Climber.kWinchEndpointUp);
  }


  /**
   * Gets the current corresponding to the torque output of the winch motor.
   * @return The absolute value of the torque current of the winch motor, between 0 and 327.68
   */
  public double getWinchCurrent(){
    return Math.abs(m_winchMotor.getTorqueCurrent().getValueAsDouble());
  }

  
  /**
   * Returns if the winch is hitting the lower limit.
    * @return True if the winch is within a cm of the lower limit, false otherwise.
   */
  public boolean isWinchAtLowerLimit(){
    return Math.abs(getWinchPosition() - Constants.Climber.kWinchEndpointDown) < 1.0;
  }



  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb Limit Switch", this.getSwitchState());
    SmartDashboard.putNumber("Extender Speed", this.getExtenderVelocity());
    SmartDashboard.putNumber("Winch Speed", this.getWinchVelocity());
    SmartDashboard.putNumber("Winch Current", this.getWinchCurrent());
    SmartDashboard.putBoolean("Is Winch at Lower Limit", this.isWinchAtLowerLimit());
    SmartDashboard.putNumber("Extender Position Cm", this.getExtenderPosition());
    SmartDashboard.putNumber("Winch Position Cm", this.getWinchPosition());
    SmartDashboard.putNumber("Extender Position Raw", m_extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Winch Position Raw", m_winchMotor.getPosition().getValueAsDouble());

  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {

  }

}
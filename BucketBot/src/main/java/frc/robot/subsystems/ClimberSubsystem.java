package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// neo pulls down
// falcon controls climbing

public class ClimberSubsystem extends SubsystemBase {
  
  // Physical devices
  private final CANSparkMax m_extenderMotor;

  private double rotationConstant;
  // ClimberSubsystem constructor
  public ClimberSubsystem() {
    super();

    // Create physical devices
    m_extenderMotor = new CANSparkMax(Constants.Climber.kExtenderMotorID, MotorType.kBrushless);

    // reset motor configurations to avoid uncertain configurations
    m_extenderMotor.restoreFactoryDefaults();
    m_extenderMotor.setIdleMode(IdleMode.kBrake);

    rotationConstant = 1;
  }

  /**
   * Spin the extender motor directly with no limits.
   * @param power The speed to spin the extender motor at, from -1.0 to 1.0
   */
  public void spinExtender(double power){
    m_extenderMotor.set(power);
  }

/**
   * Returns the position of the extender motor
   * @return Position of the extender motor in centimeters
   */
  public double getExtenderPosition(){
    return m_extenderMotor.getEncoder().getPosition();
  }

  //Returns the heading of the chair relative to the robot.
  public double getChairHeading(){
    double currentPosition = this.getExtenderPosition();
    return currentPosition * Math.PI/this.rotationConstant;
  }

  public void resetChair(){
    m_extenderMotor.getEncoder().setPosition(0);
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


  

  

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extender Position Cm", this.getExtenderPosition());
    SmartDashboard.putNumber("Extender Position Raw", m_extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Chair Position", this.getChairHeading());
    
  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {

  }

}
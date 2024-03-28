package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  // Declare Falcon controllers
  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;

  final VelocityVoltage m_velocity = new VelocityVoltage(0);

  
  // ShooterSubsystem constructor
  public ShooterSubsystem() {
    super();
    m_shooterMotor1 = new TalonFX(Constants.Shooter.kShooterMotorID1);
    m_shooterMotor2 = new TalonFX(Constants.Shooter.kShooterMotorID2);

    m_shooterMotor1.getConfigurator().apply(new TalonFXConfiguration()); // Resets the motor configurations
    m_shooterMotor2.getConfigurator().apply(new TalonFXConfiguration());

    m_shooterMotor1.setInverted(true);

    // Allows editing shooter speeds for testing purposes
    SmartDashboard.putNumber("Amp Speed", Constants.Shooter.kAmpSpeed);
    SmartDashboard.putNumber("Speaker Speed", Constants.Shooter.kSpeakerSpeedRPS);

    // Velocity control for shooter motor using FF and P controller
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.Shooter.kShooterFFS; // Voltage Required to move the motors at all. 
    slot0Configs.kV = Constants.Shooter.kShooterFFV; // Same as kF - V per rot/s <-- Calculated from applied voltage/ spin velocity
    slot0Configs.kP = Constants.Shooter.kShooterKP; // Proportional factor needs tuning
    slot0Configs.kI = Constants.Shooter.kShooterKI;  // Integral factor useful if not getting to set point
    slot0Configs.kD = Constants.Shooter.kShooterKD; // derivative factor to prevent overshoot
    //Put these control values into Slot0 in the talonFx controller- with a 50 ms overflow limit.
    m_shooterMotor1.getConfigurator().apply(slot0Configs, 0.050);
    m_shooterMotor2.getConfigurator().apply(slot0Configs, 0.050);
  }


  /**
   * Spin the motors at the given power
   * @param power Double between -1.0 and 1.0
   */
  public void spin(double power){

    SmartDashboard.putNumber("Shooter Commanded Power", power);
    
    // Real motors
    m_shooterMotor1.set(power);
    m_shooterMotor2.set(power);
  }


  // Default subsystem methods

//Spin Shooter method with Velocity Control - this code should spin at 50 revolutions per second

/**
 * Spin the shooter at the given speed.
 * @param speed Speed in rotations per second
 */
public void spinAt(double speed){
  m_velocity.Slot = 0;
  m_shooterMotor1.setControl(m_velocity.withVelocity(speed));
  m_shooterMotor2.setControl(m_velocity.withVelocity(speed));

}

  // Unused
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
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


  // Called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity 1 (RPS)", m_shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity 2 (RPS)", m_shooterMotor2.getVelocity().getValueAsDouble());
  }


  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
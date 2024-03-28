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
    SmartDashboard.putNumber("Speaker Speed", Constants.Shooter.kSpeakerSpeed);

    //Shooter Motor Spin Velocity Control Code:

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.05; // Voltage Required to move the motors at all. 
    slot0Configs.kV = 0.12; // Same as kF - V per rot/s <-- Calculated from applied voltage/ spin velocity
    slot0Configs.kP = 0.11; // Proportional factor needs tuning
    slot0Configs.kI = 0.0;  // Integral factor useful if not getting to set point
    slot0Configs.kD = 0.00;
    //Put these control values into Slot0 in the talonFx controller- with a 50 ms overflow limit.
    m_shooterMotor1.getConfigurator().apply(slot0Configs, 0.050);
    m_shooterMotor2.getConfigurator().apply(slot0Configs, 0.050);
  }


  /**
   * Spin the motors at the given speed
   * @param speed Double between -1.0 and 1.0
   */
  public void spin(double speed){

    SmartDashboard.putNumber("Shooter Commanded Power", speed);
    
    // Real motors
    m_shooterMotor1.set(speed);
    m_shooterMotor2.set(speed);
  }


  // Default subsystem methods

//Spin Shooter method with Velocity Control - this code should spin at 50 revolutions per second

public void spinShooterWithControl(){
  m_velocity.Slot = 0;
  m_shooterMotor1.setControl(m_velocity.withVelocity(50));
  m_shooterMotor2.setControl(m_velocity.withVelocity(50));

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
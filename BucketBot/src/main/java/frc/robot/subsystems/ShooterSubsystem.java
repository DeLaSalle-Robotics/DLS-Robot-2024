package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  // Declare Falcon controllers
  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;

  
  // ShooterSubsystem constructor
  public ShooterSubsystem() {
    super();
    m_shooterMotor1 = new TalonFX(Constants.Shooter.kShooterMotorID1);
    m_shooterMotor2 = new TalonFX(Constants.Shooter.kShooterMotorID2);

    m_shooterMotor1.setInverted(true);

    // Allows editing shooter speeds for testing purposes
    SmartDashboard.putNumber("Amp Speed", Constants.Shooter.kAmpSpeed);
    SmartDashboard.putNumber("Speaker Speed", Constants.Shooter.kSpeakerSpeed);
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
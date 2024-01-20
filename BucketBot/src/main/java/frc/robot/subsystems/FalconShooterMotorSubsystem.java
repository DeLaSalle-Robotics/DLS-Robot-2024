package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FalconShooterMotorSubsystem extends SubsystemBase {

  //private final CANSparkMax m_shooterMotor = new CANSparkMax(Constants.OperatorConstants.kShooterMotorID, MotorType.kBrushless);
  // Declare Falcon controllers
  private final TalonFX m_shooterMotor1 = new TalonFX(Constants.Shooter.kShooterMotorID1);
  private final TalonFX m_shooterMotor2 = new TalonFX(Constants.Shooter.kShooterMotorID2);

  // Simulated Talons
  private final TalonFXSimState m_simMotor1 = m_shooterMotor1.getSimState();
  private final TalonFXSimState m_simMotor2 = m_shooterMotor2.getSimState();

  
  // FalconShooterMotorSubsystem constructor
  public FalconShooterMotorSubsystem() {
    super();
    // Simulate the battery supply voltage
    m_simMotor1.setSupplyVoltage(12);
    m_simMotor2.setSupplyVoltage(12);
  }


  /**
   * Spin the motors at the given speed
   * @param speed Double between -1.0 and 1.0
   */
  public void spin(double speed){

    // Real motors
    m_shooterMotor1.set(speed);
    m_shooterMotor2.set(-speed);
    
    // Simulated motors
    m_simMotor1.setRotorVelocity(speed);
    m_simMotor2.setRotorVelocity(-speed);

    // Post speed to SmartDashboard
    SmartDashboard.putNumber("speed", speed);
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
  public void periodic() {}


  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
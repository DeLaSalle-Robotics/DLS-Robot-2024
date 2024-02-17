package frc.robot.subsystems;

// import frc.robot.Constants;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class NEOShooterMotorSubsystem extends SubsystemBase {

  // Declare NEO controller
  // private final CANSparkMax m_NEOShooterMotor = new CANSparkMax(Constants.Shooter.kShooterMotorID1, MotorType.kBrushless);
  

  // NEOShooterMotorSubsystem constructor
  public NEOShooterMotorSubsystem() {
    super();
    // m_NEOShooterMotor.setSmartCurrentLimit(5);
    SmartDashboard.putNumber("Speed", 0);
  }


  /**
   * Spin the motors at the given speed
   * @param speed Double between -1.0 and 1.0
   */
  public void spin(double speed){

    // Real motors
    // m_NEOShooterMotor.set(speed);

    // Post speed to SmartDashboard
    // SmartDashboard.putNumber("speed", speed);

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
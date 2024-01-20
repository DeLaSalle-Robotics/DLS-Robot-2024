package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

  // Declare a SparkMax as the intake motor controller
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.FalconShooterConstants.kIntakeMotorID, MotorType.kBrushless);


  // IntakeSubsystem constructor
  public IntakeSubsystem() {
    super();
  }


  // Spin the intake motor at the given speed
  // Speed should be between -1.0 and 1.0
  public void spin(double speed) {
    m_IntakeMotor.set(speed);
  }


  // Default subsystem methods


  // Unused
  public Command exampleMethodCommand() {
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


  // This method will be called once per scheduler run
  @Override
  public void periodic() {}


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}
  
}

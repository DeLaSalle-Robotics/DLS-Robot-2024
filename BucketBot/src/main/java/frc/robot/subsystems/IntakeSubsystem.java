package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.FalconShooterConstants.kIntakeMotorID, MotorType.kBrushless);
  /**
  FIX CHANNEL ASAP
   */
  @Deprecated
  private final DigitalInput m_LimitSwitch = new DigitalInput(0);

  public IntakeSubsystem() {
    super();
  }
  public boolean testLimitSwitch() {
    return m_LimitSwitch.get();
  }

  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void spin(double speed) {
    m_IntakeMotor.set(speed);
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

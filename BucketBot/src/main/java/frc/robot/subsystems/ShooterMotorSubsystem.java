// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterMotorSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooterMotor = new CANSparkMax(0, MotorType.kBrushless);

  /** Creates a new ShooterMotorSubsystem. */
  public ShooterMotorSubsystem() {
    super();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void spin(double speed){
    /*double voltage = Math.abs(speed) * Constants.kShooterMaxVolts;
    boolean inverted = speed < 0;
    System.out.println(voltage + " // " + inverted);
    m_shooterMotor.setInverted(inverted);
    m_shooterMotor.setVoltage(voltage);
    */
    m_shooterMotor.set(speed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
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

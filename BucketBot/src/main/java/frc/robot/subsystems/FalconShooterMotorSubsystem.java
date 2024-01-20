// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconShooterMotorSubsystem extends SubsystemBase {

  //private final CANSparkMax m_shooterMotor = new CANSparkMax(Constants.OperatorConstants.kShooterMotorID, MotorType.kBrushless);
  private final TalonFX m_shooterMotor1 = new TalonFX(Constants.FalconShooterConstants.kShooterMotorID1);
  private final TalonFX m_shooterMotor2 = new TalonFX(Constants.FalconShooterConstants.kShooterMotorID2);

  private final TalonFXSimState m_simMotor1 = m_shooterMotor1.getSimState();
  private final TalonFXSimState m_simMotor2 = m_shooterMotor2.getSimState();

  /** Creates a new ShooterMotorSubsystem. */
  public FalconShooterMotorSubsystem() {
    super();
    m_simMotor1.setSupplyVoltage(12);
    m_simMotor2.setSupplyVoltage(12);
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
    m_shooterMotor1.set(speed);
    m_shooterMotor2.set(-speed);
    
    m_simMotor1.setRotorVelocity(speed);
    m_simMotor2.setRotorVelocity(-speed);

    SmartDashboard.putNumber("speed", speed);
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

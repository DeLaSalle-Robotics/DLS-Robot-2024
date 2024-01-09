// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/** An example command that uses an example subsystem. */
public class ArmVoltQuasistatic extends CommandBase {
  private final Arm m_Arm;
  private boolean armDirection;
  private double interval;
  private double currentVoltage;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmVoltQuasistatic(Arm subsystem) {
    m_Arm = subsystem;
    addRequirements(m_Arm);
    interval = 0.01;
    currentVoltage = 0;
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    armDirection = SmartDashboard.getBoolean("Arm Calibration", false);
    if (armDirection) {
      interval = -0.01;
    } else {interval = 0.01;}
  } 
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentVoltage = currentVoltage - interval;
    m_Arm.armSetVolts(currentVoltage);
    SmartDashboard.putNumber("Arm Angle", m_Arm.ArmAngle());
    SmartDashboard.putNumber("Arm Rate", m_Arm.ArmVelocity());
    SmartDashboard.putNumber("Arm Volts", currentVoltage);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentVoltage = 0;
    m_Arm.armSetVolts(0.0);
  }

  /*// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }*/
}

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {

  private final IntakeSubsystem m_Intake;
  private final DoubleSupplier m_speed;

  /**
   * Spin the intake at the given speed until the limit switch detects something.
   * @param speed Speed to spin the intake at, between -1.0 and 1.0
   * @param subsystem Intake subsystem
   */
  public Intake(DoubleSupplier speed, IntakeSubsystem subsystem) {
    m_Intake = subsystem;
    m_speed = speed;
    addRequirements(m_Intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.spin(m_speed.getAsDouble());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The intake stops spinning when the command ends.
    m_Intake.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Intake.getEncoderRate() <= -300.0);
  }
}

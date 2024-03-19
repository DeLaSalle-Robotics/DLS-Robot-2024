package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class Intake extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final DoubleSupplier m_speed;

  private boolean movingForward;


  /**
   * Spin the intake at the given speed until the loose encoder detects something.
   * @param subsystem IntakeSubsystem
   * @param speed Speed to spin the intake at, between -1.0 and 1.0
   */
  public Intake(IntakeSubsystem subsystem, DoubleSupplier speed) {
    m_IntakeSubsystem = subsystem;
    m_speed = speed;
    addRequirements(m_IntakeSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    movingForward = (m_speed.getAsDouble() > 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.spinDirect(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The intake stops spinning when the command ends.
    m_IntakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return movingForward && m_IntakeSubsystem.noteDetected();
  }
}

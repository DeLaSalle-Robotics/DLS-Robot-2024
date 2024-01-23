package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FalconShooterMotorSubsystem;

/** Spins the falcon shooter motor until it is done shooting */
public class Shooter extends Command {
  private final FalconShooterMotorSubsystem m_shooter;

  private final DoubleSupplier m_speed;

  public Shooter(DoubleSupplier speed, FalconShooterMotorSubsystem subsystem) {
    m_speed = speed;
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spin(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

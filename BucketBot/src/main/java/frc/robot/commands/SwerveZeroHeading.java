package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SwerveZeroHeading extends Command {

  private final SwerveSubsystem m_swerve;

  public SwerveZeroHeading(SwerveSubsystem subsystem) {
    m_swerve = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.zeroHeading();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

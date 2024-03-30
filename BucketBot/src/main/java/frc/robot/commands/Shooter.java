package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class Shooter extends Command {

  // Command parameters
  private final ShooterSubsystem m_ShooterSubsystem;

  /**
   * Spin the shooter. The speeds are defined in {@link Constants}.
   * @param subsystem ShooterSubsystem
   */
  public Shooter(ShooterSubsystem subsystem) {
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_ShooterSubsystem.spinAt(Constants.Shooter.kShooterSpeedRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ShooterSubsystem.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

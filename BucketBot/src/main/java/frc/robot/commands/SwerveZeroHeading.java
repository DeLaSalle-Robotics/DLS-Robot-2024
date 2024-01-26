package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class SwerveZeroHeading extends Command {

  private final SwerveSubsystem m_swerve;

  /**
   * Resets the gyro to re-orient the robot on the field due to drift..
   * @param subsystem Swerve subsystem
   */
  public SwerveZeroHeading(SwerveSubsystem subsystem) {
    m_swerve = subsystem;
    addRequirements(m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Reset the gyro
    m_swerve.zeroHeading();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End this command immediately
    return true;
  }
}

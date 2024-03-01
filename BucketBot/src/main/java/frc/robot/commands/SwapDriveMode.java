package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwapDriveMode extends Command {

  private final SwerveSubsystem m_swerve;
  private Rotation2d oldHeading;

  /**
   * Changes the drive mode from field-oriented to robot-oriented when scheduled. Changes back to field-oriented when ended.
   * @param swerve SwerveSubsystem
   */
  public SwapDriveMode(SwerveSubsystem swerve) {
    // Assign subsystem
    m_swerve = swerve;
    addRequirements(m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Store heading
    oldHeading = m_swerve.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.zeroGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.resetOdometry(new Pose2d(
      m_swerve.getPose().getTranslation(),
      oldHeading
    ));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

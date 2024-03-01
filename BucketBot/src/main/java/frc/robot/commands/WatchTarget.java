package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class WatchTarget extends Command {


  private final VisionSubsystem m_vision;

  private final int aprilTagID;

  /**
   * Force the robot to watch an april tag of the given fiducial ID.
   * @param aprilID ID of the april tag to watch, from 1-16.
   * @param vision VisionSubsystem.
   */
  public WatchTarget(int aprilTagID, VisionSubsystem vision) {

    // Assign subsystems
    m_vision = vision;

    // Assign april tag ID
    this.aprilTagID = aprilTagID;

    addRequirements(m_vision);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.watchAprilTag(aprilTagID);
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

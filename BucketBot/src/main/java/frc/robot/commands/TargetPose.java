package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetPose extends Command {


  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private double diff;
  /**
   * Force the robot to watch an april tag of the given fiducial ID.
   * @param subsystem VisionSubsystem
   * @param aprilID ID of the april tag to watch, from 1-16.
   */
  public TargetPose(VisionSubsystem visSubsystem, SwerveSubsystem swerveSubsystem) {

    // Assign subsystems
    m_VisionSubsystem = visSubsystem;
    m_SwerveSubsystem = swerveSubsystem;
    diff = 0;
    addRequirements(m_VisionSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double target = m_VisionSubsystem.getPoseViaTag();
     diff = target - Math.PI;
     if(Math.abs(diff) > Units.degreesToRadians(10)) {
      Translation2d Translation2d = new Translation2d();
      double drive = diff * 1.0; //Drive correction for auto aim.
      m_SwerveSubsystem.drive(Translation2d , drive, false);
     } 
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(diff) < Units.degreesToRadians(10)){
      return true;
    } else {
    return false;
    }
  }
}

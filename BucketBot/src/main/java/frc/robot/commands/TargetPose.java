package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class TargetPose extends Command {


  private final VisionSubsystem m_VisionSubsystem;
  
  /**
   * Force the robot to watch an april tag of the given fiducial ID.
   * @param subsystem VisionSubsystem
   * @param aprilID ID of the april tag to watch, from 1-16.
   */
  public TargetPose(VisionSubsystem subsystem) {

    // Assign subsystems
    m_VisionSubsystem = subsystem;

    addRequirements(m_VisionSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_VisionSubsystem.getPoseViaTag();
      
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

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetPose extends Command {


  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private double diff;
  NetworkTable table;

  DoubleSubscriber TargetYawSub;
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
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    TargetYawSub = table.getDoubleTopic("Target_Yaw").subscribe(0);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double target = TargetYawSub.get();
     if(Math.abs(target) > (5)) {
      double drive = - Units.degreesToRadians(target) * 2 ; //Drive correction for auto aim.
      System.out.println("Turn num " + drive);
      m_SwerveSubsystem.driveCommand(
      () -> 0,
      () -> 0,
      () -> drive,
      () -> 0);
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

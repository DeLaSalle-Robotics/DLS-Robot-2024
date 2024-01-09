package frc.robot.commands;

// Unused imports
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Creates a new AlignToTarget. **/
public class AlignToTarget extends CommandBase {
  DrivetrainSubsystem m_drive;

  // Target rotation and a boolean to tell the direction to turn
  double targetRotation;
  boolean isRight = true;

  public AlignToTarget(double targetAngleDegrees, DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.'
    m_drive = _drive;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationSpeed = 0.5;//m_drive.targetRotation(targetRotation);
//Should determine Ks and add/substract it to the output. Just to get it spinning
    m_drive.driveVolts(-rotationSpeed-3, rotationSpeed+3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(m_drive.getHeading() - targetRotation);
    if (Constants.verbose) {
      SmartDashboard.putNumber("Heading Error", error);
    }
    m_drive.driveVolts(0, 0);
    return error < Constants.angleTolerance;
  }
  
}
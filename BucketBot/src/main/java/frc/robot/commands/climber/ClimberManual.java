package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberManual extends Command {

  private final ClimberSubsystem m_Climber;
  private final boolean m_moveUp;

  /**
   * Move the climber up or down, depending on the starting position.
   * @param subsystem Climber subsystem
   */
  public ClimberManual(boolean moveUp, ClimberSubsystem subsystem) {
    m_Climber = subsystem;
    m_moveUp = moveUp;
    addRequirements(m_Climber);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get motor velocities
    double extenderVelocity = m_Climber.getExtenderVelocity();
    double climberVelocity = m_Climber.getClimberVelocity();

    // Calculate errors
    double extenderError = 0.0;
    double climberError = 0.0;
    if (m_moveUp){
      extenderError = Constants.Climber.kExtenderMotorVelocityRPM - extenderVelocity;
      climberError = -Constants.Climber.kClimberMotorVelocityRPM - climberVelocity;
    } else {
      extenderError = -Constants.Climber.kExtenderMotorVelocityRPM - extenderVelocity;
      climberError = Constants.Climber.kClimberMotorVelocityRPM - climberVelocity;
    }
 
    SmartDashboard.putNumber("extenderError", extenderError);
    SmartDashboard.putNumber("climberError", climberError);

    // Retrieve kP values
    double kPExtenderDown = SmartDashboard.getNumber("kPExtenderDown", Constants.Climber.kPExtenderDown);
    double kPClimberDown = SmartDashboard.getNumber("kPClimberDown", Constants.Climber.kPClimberDown);
    double kPExtenderUp = SmartDashboard.getNumber("kPExtenderUp", Constants.Climber.kPExtenderUp);
    double kPClimberUp = SmartDashboard.getNumber("kPClimberUp", Constants.Climber.kPClimberUp);

    // Set motor velocities
    if (m_moveUp){
      // Move up
      m_Climber.spinMotors(
        Constants.Climber.kExtenderFeedForwardUp + (kPExtenderUp * extenderError), 
        Constants.Climber.kClimberFeedForwardUp - (kPClimberUp * climberError)
      );
    } else {
      // Move down
      m_Climber.spinMotors(
        Constants.Climber.kExtenderFeedForwardDown - (kPExtenderDown * extenderError), 
        Constants.Climber.kClimberFeedForwardDown + (kPClimberDown * climberError)
      );
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_Climber.spinMotors(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
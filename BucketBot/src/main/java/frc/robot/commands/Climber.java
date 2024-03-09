package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Climber extends Command {

  private final ClimberSubsystem m_Climber;

  private boolean movingUp = true;

  /**
   * Move the climber up or down, depending on the starting position.
   * @param subsystem Climber subsystem
   */
  public Climber(ClimberSubsystem subsystem) {
    m_Climber = subsystem;
    addRequirements(m_Climber);
    SmartDashboard.putBoolean("movingUp", movingUp);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Check the state of the climber
    movingUp = (m_Climber.getExtenderPosition() < -80 && m_Climber.getClimberPosition() > 60);
    SmartDashboard.putBoolean("movingUp", movingUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Read limit switch
    if(m_Climber.getSwitchState()){
      m_Climber.setSoftLimits();
    }

    // Get motor velocities
    double extenderVelocity = m_Climber.getExtenderVelocity();
    double climberVelocity = m_Climber.getClimberVelocity();

    // Calculate errors
    double extenderError;
    double climberError;
    if (movingUp) {
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
    if (movingUp){
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

    // Shortens conditionals
    double extenderPos = m_Climber.getExtenderPosition();
    double climberPos = m_Climber.getClimberPosition();

    // If moving up and at least one motor reaches its endpoint, stop the command
    if ((movingUp) && (extenderPos >= Constants.Climber.kExtenderEndpointUp || climberPos <= Constants.Climber.kClimberEndpointUp)){
      return true;

    // If moving down and at least one motor reaches its endpoint, stop the command
    } else if ((!movingUp) && (extenderPos <= Constants.Climber.kExtenderEndpointDown || climberPos >= Constants.Climber.kClimberEndpointDown)){
      return true;

    // The motors have not reached their endpoint yet, continue the command
    } else {
      return false;
    }
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
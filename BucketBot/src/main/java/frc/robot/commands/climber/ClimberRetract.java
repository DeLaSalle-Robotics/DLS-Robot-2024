package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberRetract extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;

  /**
   * 
   * @param subsystem ClimberSubsystem
   */
  public ClimberRetract(ClimberSubsystem subsystem) {
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spin the extender with no limits
    m_ClimberSubsystem.spinExtenderAt(-Constants.Climber.kExtenderTargetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ClimberSubsystem.spinExtenderAt(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ClimberSubsystem.getSwitchState();
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
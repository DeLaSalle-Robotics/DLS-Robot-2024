package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class ClimberSwap extends Command {


  private final ClimberSubsystem m_Climber;


  /**
   * Swaps the active motor of the climber when using test mode.
   * <p><b>This is only used in test mode.</b>
   * @param climber Climber Subsystem
   */
  public ClimberSwap(ClimberSubsystem climber) {
    m_Climber = climber;
    addRequirements(m_Climber);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climber.swapActiveMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No execute, since this is a one-time action
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
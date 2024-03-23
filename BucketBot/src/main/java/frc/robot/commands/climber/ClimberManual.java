package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberManual extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final boolean m_movingUp;

  /**
   * 
   * @param subsystem ClimberSubsystem
   * @param movingUp
   */
  public ClimberManual(ClimberSubsystem subsystem, boolean movingUp) {
    m_ClimberSubsystem = subsystem;
    m_movingUp = movingUp;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If the limit switch is hit, reset the extender encoder
    if(m_ClimberSubsystem.getSwitchState()){
      m_ClimberSubsystem.setExtenderLowerLimit();
    }



    // If moving in reverse and the extender is at position 0, don't move
    if(!m_movingUp && m_ClimberSubsystem.getExtenderPosition() <= 0){
      m_ClimberSubsystem.spinExtender(0.0);
      m_ClimberSubsystem.spinWinch(0.0);

    // If moving forward and the extender has exceeded the endpoint, don't move
    } else if (m_movingUp && m_ClimberSubsystem.getExtenderPosition() >= Constants.Climber.kExtenderDistanceCm){
      m_ClimberSubsystem.spinExtender(0.0);
      m_ClimberSubsystem.spinWinch(0.0);

    // Otherwise, move
    } else {
      double direction = m_movingUp? 1.0 : -1.0;
      m_ClimberSubsystem.spinExtender(0.0);
      m_ClimberSubsystem.spinWinch(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ClimberSubsystem.spinExtender(0.0);
    m_ClimberSubsystem.spinWinch(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
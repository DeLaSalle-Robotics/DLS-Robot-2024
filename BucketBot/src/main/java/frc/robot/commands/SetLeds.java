package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;

public class SetLeds extends Command {
  
  private final LedSubsystem m_led = new LedSubsystem();
  private int m_start;
  private int m_end;
  private int m_red;
  private int m_green;
  private int m_blue;
  
  public SetLeds(int start, int end, int red, int green, int blue){
    m_start = start;
    m_end = end;
    m_red = red;
    m_green = green;
    m_blue = blue;
    addRequirements(m_led);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.setSection(m_start, m_end, m_red, m_green, m_blue);
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

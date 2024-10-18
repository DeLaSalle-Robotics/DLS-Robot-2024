package frc.robot.commands;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRED extends Command {
    Color kRed;
private LEDSubsystem m_LEDSubsystem;

public void LEDRED(LEDSubsystem ledSubsystem) {
    m_LEDSubsystem = ledSubsystem;
}





@Override
  public void execute() {
  
    m_LEDSubsystem.set(kRed);
    
}
}
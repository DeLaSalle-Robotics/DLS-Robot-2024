package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class LED extends Command {

  // Command parameters
  private final LEDSubsystem m_LEDSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  /**
   * Control the LEDs based on what other subsystems are doing
   * @param ledSubsystem LEDSubsystem
   * @param intakeSubsystem IntakeSubsystem
   */
  public LED(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
    m_LEDSubsystem = ledSubsystem;
    m_IntakeSubsystem = intakeSubsystem;

    addRequirements(m_LEDSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * Control the LEDs based on the state of the other subsystems
   * <p>Blue: Intake has a note
   * <p>Off: Otherwise
   */
  @Override
  public void execute() {
    if(m_IntakeSubsystem.hasNote()){
      m_LEDSubsystem.set(Color.kBlue);
    } else {
      m_LEDSubsystem.off();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Turn off the LEDs
    m_LEDSubsystem.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

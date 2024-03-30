package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LEDSubsystem;


public class LEDTest extends Command {

  // Command parameters
  private final LEDSubsystem m_LEDSubsystem;
  private final Color m_color;

  /**
   * Set the LED to a specific color for testing purposes
   * @param led_subsystem LEDSubsystem
   * @param color Color to set the LEDs to
   */
  public LEDTest(LEDSubsystem ledSubsystem, Color color) {
    m_LEDSubsystem = ledSubsystem;
    m_color = color;

    addRequirements(m_LEDSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDSubsystem.set(m_color);
  }

  /**
   * Control the LEDs based on the state of the other subsystems
   * Green: Shooter is at speed
   * Blue: Intake has a note
   * Red: If the climber is maxed out
   * Off: Otherwise
   */
  @Override
  public void execute() {
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
    return (m_color == Color.kBlack);
  }
}

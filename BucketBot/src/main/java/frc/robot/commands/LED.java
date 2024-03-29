package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class LED extends Command {

  // Command parameters
  private final LEDSubsystem m_LEDSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;

  /**
   * Control the LEDs based on what other subsystems are doing
   * @param led_subsystem LEDSubsystem
   * @param shooter_subsystem ShooterSubsystem
   * @param intake_subsystem IntakeSubsystem
   */
  public LED(LEDSubsystem led_subsystem, ShooterSubsystem shooter_subsystem, IntakeSubsystem intake_subsystem, ClimberSubsystem climber_subsystem) {
    m_LEDSubsystem = led_subsystem;
    m_ShooterSubsystem = shooter_subsystem;
    m_IntakeSubsystem = intake_subsystem;
    m_ClimberSubsystem = climber_subsystem;

    addRequirements(m_LEDSubsystem);
    addRequirements(m_ShooterSubsystem);
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * Control the LEDs based on the state of the other subsystems
   * Green: Shooter is at speed
   * Blue: Intake has a note
   * Red: If the climber is maxed out
   * Off: Otherwise
   */
  @Override
  public void execute() {
    if(m_ShooterSubsystem.atSpeed()){
      m_LEDSubsystem.set(Colors.GREEN);
    } else if(m_IntakeSubsystem.hasNote()){
      m_LEDSubsystem.set(Colors.Blue);
    } else if(m_ClimberSubsystem.isMaxedOut()){
        m_LEDSubsystem.set(Colors.RED);
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

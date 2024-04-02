package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  public LED(LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
    m_LEDSubsystem = ledSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_ClimberSubsystem = climberSubsystem;

    addRequirements(m_LEDSubsystem);
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
      m_LEDSubsystem.set(Color.kGreen);
    } else if(m_IntakeSubsystem.hasNote()){
      m_LEDSubsystem.set(Color.kBlue);
    } else if(m_ClimberSubsystem.isWinchAtLowerLimit()){
        m_LEDSubsystem.set(Color.kGreen);
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

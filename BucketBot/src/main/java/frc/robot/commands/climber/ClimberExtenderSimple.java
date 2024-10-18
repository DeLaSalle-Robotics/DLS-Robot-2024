package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberExtenderSimple extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final DoubleSupplier m_power;

  /**
   * 
   * @param subsystem ClimberSubsystem
   * @param power
   */
  public ClimberExtenderSimple(ClimberSubsystem subsystem, DoubleSupplier power) {
    m_ClimberSubsystem = subsystem;
    m_power = power;
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
     m_ClimberSubsystem.spinExtender(
      m_power.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop extender with brake
    m_ClimberSubsystem.spinExtender(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
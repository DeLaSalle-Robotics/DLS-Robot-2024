package frc.robot.commands.climber;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberWinchSimple extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final DoubleSupplier m_power;

  /**
   * 
   * @param subsystem ClimberSubsystem
   * @param power
   */
  public ClimberWinchSimple(ClimberSubsystem subsystem, DoubleSupplier power) {
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

    // Spin winch motor with soft limits
    m_ClimberSubsystem.spinWinch(m_power.getAsDouble(),
        Constants.Climber.kWinchEndpointDown,
        Constants.Climber.kWinchEndpointUp);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop winch
    m_ClimberSubsystem.spinWinch(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
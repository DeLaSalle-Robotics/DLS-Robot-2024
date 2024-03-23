package frc.robot.commands.climber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberTest extends Command {


  private final ClimberSubsystem m_ClimberSubsystem;
  private final DoubleSupplier m_extenderPower;
  private final DoubleSupplier m_winchPower;

  /**
   * Allows moving the climber motors individually with joysticks
   * <p><i>This should only be used in test mode.</i>
   * @param subsystem ClimberSubsystem
   * @param extenderPower Power to send to the extender motor, between -1.0 and 1.0
   * @param winchPower Power to send to the winch motor, between -1.0 and 1.0
   */
  public ClimberTest(ClimberSubsystem subsystem, DoubleSupplier extenderPower, DoubleSupplier winchPower) {
    m_ClimberSubsystem = subsystem;
    m_extenderPower = extenderPower;
    m_winchPower = winchPower;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Power of each motor
    double extenderPower = m_extenderPower.getAsDouble();
    double winchPower = m_winchPower.getAsDouble();

    // Extender is moving down and limit switch is down, stop the motor
    if(extenderPower < 0 && m_ClimberSubsystem.getSwitchState()){
      m_ClimberSubsystem.spinExtender(0.0);

    // Moving up, limit switch state is not needed
    } else {
      m_ClimberSubsystem.spinExtender(extenderPower);
    }

    // Always spin the winch anyways
    m_ClimberSubsystem.spinWinch(winchPower);
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
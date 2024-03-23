package frc.robot.commands.climber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberTest extends Command {


  private final ClimberSubsystem m_ClimberSubsystem;
  private final DoubleSupplier m_power;
  private final BooleanSupplier m_extender;

  /**
   * Allows moving the climber motors individually with joysticks
   * <p><i>This should only be used in test mode.</i>
   * @param subsystem ClimberSubsystem
   * @param power Power to send to the motor, between -1.0 and 1.0
   * @param extender Which motor to use. True uses the extender, false uses the winch.
   */
  public ClimberTest(ClimberSubsystem subsystem, DoubleSupplier power, BooleanSupplier extender) {
    m_ClimberSubsystem = subsystem;
    m_power = power;
    m_extender = extender;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Power to spin the motors at
    double power = m_power.getAsDouble();

    // Determines which motor to use
    boolean usingExtender = m_extender.getAsBoolean();

    // Using extender
    if(usingExtender){

      // Moving down and limit switch is down, stop the motors
      if(power < 0 && m_ClimberSubsystem.getSwitchState()){
        m_ClimberSubsystem.spinExtender(0.0);
        m_ClimberSubsystem.spinWinch(0.0);

      // Moving up, limit switch state is not needed
      } else {
        m_ClimberSubsystem.spinExtender(power);
        m_ClimberSubsystem.spinWinch(0.0);
      }

    // Using winch
    } else {
      m_ClimberSubsystem.spinExtender(0.0);
      m_ClimberSubsystem.spinWinch(power);
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
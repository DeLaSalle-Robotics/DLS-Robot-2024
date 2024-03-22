package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberTest extends Command {


  private final ClimberSubsystem m_ClimberSubsystem;

  private final DoubleSupplier m_speed;


  /**
   * Allows moving the climber motors individually.
   * <p><b>This should only be activated in test mode.</b>
   * @param subsystem ClimberSubsystem
   * @param speed Speed of the active motor. Check SmartDashboard or Shuffleboard for live active motor data.
   */
  public ClimberTest(ClimberSubsystem subsystem, DoubleSupplier speed) {
    m_ClimberSubsystem = subsystem;
    m_speed = speed;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = m_speed.getAsDouble();

    // Extender motor
    if(SmartDashboard.getBoolean("Using Extender", true)){

      // Moving down and limit switch is down
      if(speed < 0 && m_ClimberSubsystem.getSwitchState()){
        m_ClimberSubsystem.spinExtender(0.0);
        m_ClimberSubsystem.spinWinch(0.0);

      // Else
      } else {
        m_ClimberSubsystem.spinExtender(speed);
        m_ClimberSubsystem.spinWinch(0.0);
      }

    // Winch motor
    } else {
      m_ClimberSubsystem.spinExtender(0.0);
      m_ClimberSubsystem.spinWinch(speed);
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
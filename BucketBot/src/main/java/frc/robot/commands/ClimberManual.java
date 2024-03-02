package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberManual extends Command {

  private final ClimberSubsystem m_Climber;
  
  private final DoubleSupplier m_speed1;
  private final DoubleSupplier m_speed2;

  private boolean movingUp;

  /**
   * Manually move the climber motors separately
   * @param speed1 Extender motor speed, between -1.0 and 1.0
   * @param speed2 Climber motor speed, between -1.0 and 1.0
   * @param subsystem Climber subsystem
   */
  public ClimberManual(DoubleSupplier speed1, DoubleSupplier speed2, ClimberSubsystem subsystem) {
    m_Climber = subsystem;
    m_speed1 = speed1;
    m_speed2 = speed2;
    addRequirements(m_Climber);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.spinMotors(m_speed1.getAsDouble(), m_speed2.getAsDouble());

    movingUp = (m_Climber.getExtenderPosition() < -80 && m_Climber.getClimberPosition() > 60);
    SmartDashboard.putBoolean("movingUp", movingUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_Climber.spinMotors(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
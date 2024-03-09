package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberTest extends Command {


  private final ClimberSubsystem m_Climber;

  private final double m_speed;


  /**
   * Allows moving the climber motors individually.
   * <p><b>This should only be activated in test mode.</b>
   * @param speed Speed of the active motor. Check SmartDashboard or Shuffleboard for live active motor data.
   * @param climber Climber subsystem
   */
  public ClimberTest(DoubleSupplier speed, ClimberSubsystem climber) {
    m_Climber = climber;
    m_speed = speed.getAsDouble();
    addRequirements(m_Climber);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Extender motor
    if(SmartDashboard.getBoolean("Using Extender", true)){

      // Moving down and limit switch is down
      if(m_speed < 0 && m_Climber.getSwitchState()){
        m_Climber.spinMotors(0.0, 0.0);

      // Else
      } else {
        m_Climber.spinMotors(m_speed, 0.0);
      }

    // Climb motor
    } else {
      m_Climber.spinMotors(0.0, m_speed);
    }
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
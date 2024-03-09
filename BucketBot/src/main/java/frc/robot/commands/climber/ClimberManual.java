package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberManual extends Command {

  private final ClimberSubsystem m_Climber;
  private final boolean m_MovingUp;

  public ClimberManual(boolean movingUp, ClimberSubsystem climber) {
    m_Climber = climber;
    m_MovingUp = movingUp;
    addRequirements(m_Climber);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If the limit switch is hit, reset the extender encoder
    if(m_Climber.getSwitchState()){
      m_Climber.resetExtenderEncoder();
    }



    // If moving in reverse and the extender is at position 0, don't move
    if(!m_MovingUp && m_Climber.getExtenderPosition() <= 0){
      m_Climber.spinMotors(0.0, 0.0);

    // If moving forward and the extender has exceeded the endpoint, don't move
    } else if (m_MovingUp && m_Climber.getExtenderPosition() >= Constants.Climber.kExtenderDistanceCm){
      m_Climber.spinMotors(0.0, 0.0);

    // Otherwise, move
    } else {
      double direction = m_MovingUp? 1.0 : -1.0;
      m_Climber.spinMotorsTo(
        MathUtil.clamp(m_Climber.getExtenderPosition() + Constants.Climber.kMotorOffset * direction, 0.0, Constants.Climber.kExtenderDistanceCm),
        MathUtil.clamp(m_Climber.getClimberPosition() + Constants.Climber.kMotorOffset * direction, 0.0, Constants.Climber.kClimberDistanceCm)
      );
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
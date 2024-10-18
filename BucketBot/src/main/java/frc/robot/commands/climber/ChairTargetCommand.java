package frc.robot.commands.climber;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;


public class ChairTargetCommand extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final VisionSubsystem m_VisionSubsystem;
  private boolean RedAlliance;
  /**
   * 
   * @param subsystem ClimberSubsystem
   * @param power
   */
  public ChairTargetCommand(ClimberSubsystem subsystem, VisionSubsystem visionSubsystem) {
    m_ClimberSubsystem = subsystem;
    m_VisionSubsystem = visionSubsystem;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        var alliance = DriverStation.getAlliance();
        this.RedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Spin winch motor with soft limits
    double robotRotation = m_VisionSubsystem.getPoseViaTag()%(2*Math.PI);
    if (this.RedAlliance) {
      robotRotation = robotRotation + Math.PI;
    }
    double chairRotation = m_ClimberSubsystem.getChairHeading()%Math.PI;
    double chairError = chairRotation - robotRotation;
    double maxInput = 0.2;
    if (Math.abs(chairError) > 0.2){
      if (chairError < 0) {chairError = -maxInput;}
      if (chairError > 0) {chairError = maxInput; }
    }
    m_ClimberSubsystem.spinExtender(chairError);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop winch
    m_ClimberSubsystem.spinExtender(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
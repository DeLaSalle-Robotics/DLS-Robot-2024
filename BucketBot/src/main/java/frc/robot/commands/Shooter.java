package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class Shooter extends Command {

  // Command parameters
  private final DoubleSupplier m_analogState;
  private final ShooterSubsystem m_ShooterSubsystem;

  /**
   * Spin the shooter at the given speed.
   * @param subsystem ShooterSubsystem
   * @param analogState State of the right analog trigger (RT)
   */
  public Shooter(ShooterSubsystem subsystem, DoubleSupplier analogState) {
    m_analogState = analogState;
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Under 10%, don't move
    if(m_analogState.getAsDouble() < 0.1){
      m_ShooterSubsystem.spin(0.0);

    // Between 10% and 90%, move at amp speed
    } else if (m_analogState.getAsDouble() >= 0.1 && m_analogState.getAsDouble() <= 0.9){
      m_ShooterSubsystem.spin(SmartDashboard.getNumber("Amp Speed", Constants.Shooter.kAmpSpeed));

    // Above 90%, move at speaker speed
    } else if (m_analogState.getAsDouble() > 0.9){
      m_ShooterSubsystem.spin(SmartDashboard.getNumber("Speaker Speed", Constants.Shooter.kSpeakerSpeed));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ShooterSubsystem.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

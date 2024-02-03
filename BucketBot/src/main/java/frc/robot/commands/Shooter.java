package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FalconShooterMotorSubsystem;


public class Shooter extends Command {

  // Command parameters
  private final DoubleSupplier m_speed1;
  private final DoubleSupplier m_speed2;
  private final FalconShooterMotorSubsystem m_shooter;

  /**
   * Spin the shooter motors at the given speeds.
   * @param speed1 Speed to spin the first shooter motor at, between -1.0 and 1.0.
   * @param speed2 Speed to spin the second shooter motor at, between -1.0 and 1.0.
   * @param subsystem FalconShooterMotor subsystem.
   */
  public Shooter(DoubleSupplier speed1, DoubleSupplier speed2, FalconShooterMotorSubsystem subsystem) {
    m_speed1 = speed1;
    m_speed2 = speed2;
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spin(m_speed1.getAsDouble(), m_speed2.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.spin(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

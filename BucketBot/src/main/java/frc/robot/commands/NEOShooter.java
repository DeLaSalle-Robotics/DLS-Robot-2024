package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.NEOShooterMotorSubsystem;


public class NEOShooter extends Command {

  // Command parameters
  private final DoubleSupplier m_speed;
  private final NEOShooterMotorSubsystem m_shooter;

  /**
   * Spin the shooter at the given speed.
   * @param speed Speed to spin the shooter at, between -1.0 and 1.0
   * @param subsystem NEOShooterMotor subsystem
   */
  public NEOShooter(DoubleSupplier speed, NEOShooterMotorSubsystem subsystem) {
    m_speed = speed;
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spin(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

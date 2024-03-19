package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;


public class Shooter extends Command {

  /*
  // Command parameters
  private final DoubleSupplier m_speed;
  private final FalconShooterMotorSubsystem m_ShooterSubsystem;
*/
  /**
   * Spin the shooter at the given speed.
   * @param subsystem ShooterSubsystem
   * @param speed Speed to spin the shooter at, between -1.0 and 1.0
   */
  public Shooter(ShooterSubsystem subsystem, DoubleSupplier speed) {
    /*
    m_speed = speed;
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
    */
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_ShooterSubsystem.spin(m_speed.getAsDouble());
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

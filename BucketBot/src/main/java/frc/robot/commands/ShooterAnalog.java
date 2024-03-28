package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterAnalog extends Command {

  // Command parameters
  private final DoubleSupplier m_power;
  private final ShooterSubsystem m_ShooterSubsystem;

  /**
   * Directly spin the shooter at the given speed.
   * @param subsystem ShooterSubsystem
   * @param power Power to give to the shooter motors, between -1.0 and 1.0
   */
  public ShooterAnalog(ShooterSubsystem subsystem, DoubleSupplier power) {
    m_power = power;
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.spin(m_power.getAsDouble());
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

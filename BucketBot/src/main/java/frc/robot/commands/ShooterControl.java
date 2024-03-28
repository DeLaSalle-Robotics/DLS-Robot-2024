package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterControl extends Command {

  // Command parameters

  private final ShooterSubsystem m_ShooterSubsystem;

  /**
   * Spin the shooter at the given speed.
   * @param subsystem ShooterSubsystem
   */
  public ShooterControl(ShooterSubsystem subsystem) {
      m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ShooterSubsystem.spinShooterWithControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ShooterSubsystem.spin(0.0);
    SmartDashboard.putString("Robot State", "Empty");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

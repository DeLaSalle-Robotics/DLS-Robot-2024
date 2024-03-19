package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerSubsystem;

public class Rumble extends Command {

  private final ControllerSubsystem m_ControllerSubsystem;

  private final DoubleSupplier m_rumbleSpeed;
  private final boolean m_runOnce;

  private long initialTime;


  /**
   * Activates controller rumble when scheduled, and automatically stops rumble when ended.
   * @param subsystem ControllerSubsystem
   * @param rumbleSpeed Intensity of the rumble, from 0 to 1.
   * @param onlyOnce Set to true to only run the command once. Used for "pulse" rumbles.
   */
  public Rumble(ControllerSubsystem subsystem, DoubleSupplier rumbleSpeed, boolean onlyOnce) {
    m_ControllerSubsystem = subsystem;
    m_rumbleSpeed = rumbleSpeed;
    m_runOnce = onlyOnce;
    addRequirements(m_ControllerSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = RobotController.getFPGATime();
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ControllerSubsystem.rumble(RumbleType.kBothRumble, m_rumbleSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ControllerSubsystem.rumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stops the rumble after 0.2 seconds if runOnce is true
    return m_runOnce && (RobotController.getFPGATime() - initialTime) >= (Math.pow(10.0, 5.0) * 2);
  }
}

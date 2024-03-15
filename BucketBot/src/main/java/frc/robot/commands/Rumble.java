package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {

  private final XboxController m_joystick;

  private final DoubleSupplier m_rumbleSpeed;

  /**
   * Activates controller rumble when scheduled, and automatically stops rumble when ended.
   * @param controller The controller to rumble.
   * @param rumbleSpeed Intensity of the rumble, from 0 to 1.
   */
  public Rumble(XboxController controller, DoubleSupplier rumbleSpeed) {
    m_joystick = controller;
    m_rumbleSpeed = rumbleSpeed;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_joystick.setRumble(RumbleType.kBothRumble, m_rumbleSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

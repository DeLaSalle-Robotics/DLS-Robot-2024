package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;

public class Rumble extends Command {

  private final double m_rumble1;
  private final double m_rumble2;
  private final XboxController m_joystick = new XboxController(OIConstants.kDriverControllerPort);
  
  /**
   * Set the rumble of the Xbox controller.
   * <p>It's possible to set individual rumble motor speeds, but it's usually better to simply use both at the same value.
   * <p>The rumble speed will be reset to zero when the command ends to prevent the controller from rumbling indefinitely.
   * @param rumbleL Left rumble motor, slightly more intense than the right
   * @param rumbleR Right rumble motor, slightly less intense than the left
   */
  public Rumble(double rumbleL, double rumbleR) {
    m_rumble1 = rumbleL;
    m_rumble2 = rumbleR;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For whatever reason the motors are on opposite sides than what is claimed
    m_joystick.setRumble(RumbleType.kRightRumble, m_rumble1);
    m_joystick.setRumble(RumbleType.kLeftRumble, m_rumble2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // MAKE SURE YOU DON'T DELETE THIS OR ELSE THE RUMBLE WILL CONTINUE INDEFINITELY
    m_joystick.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

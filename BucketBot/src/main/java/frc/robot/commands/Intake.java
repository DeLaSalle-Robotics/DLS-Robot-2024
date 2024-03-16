package frc.robot.commands;

import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Rumble;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends Command {

  private final IntakeSubsystem m_Intake;
  private final DoubleSupplier m_speed;

  private boolean movingForward;

  /**
   * Spin the intake at the given speed until the limit switch detects something.
   * @param speed Speed to spin the intake at, between -1.0 and 1.0
   * @param intakeSubsystem IntakeSubsystem
   */
  public Intake(DoubleSupplier speed, IntakeSubsystem intakeSubsystem) {
    m_Intake = intakeSubsystem;
    m_speed = speed;
    addRequirements(m_Intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    movingForward = (m_speed.getAsDouble() > 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.spin(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The intake stops spinning when the command ends.
    m_Intake.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return movingForward && (m_Intake.getEncoderRate() <= -300.0);
  }
}

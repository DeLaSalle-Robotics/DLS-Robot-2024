package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;


public class Intake extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final DoubleSupplier m_speed;
  private final BooleanSupplier m_isFeeding;

  private boolean movingForward;

  DoubleSubscriber ShooterSpeedSub;
  BooleanPublisher NotePub;
  /**
   * Spin the intake at the given speed until the loose encoder detects something.
   * @param subsystem IntakeSubsystem
   * @param speed Speed to spin the intake at, between -1.0 and 1.0
   * @param isFeeding Set this to true to ignore the loose encoder detection system.
   */
  public Intake(IntakeSubsystem subsystem, DoubleSupplier speed, BooleanSupplier isFeeding) {
    m_IntakeSubsystem = subsystem;
    m_speed = speed;
    m_isFeeding = isFeeding;
    addRequirements(m_IntakeSubsystem);
    
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("datatable");
  ShooterSpeedSub = table.getDoubleTopic("ShooterSpeed").subscribe(0);
  NotePub = table.getBooleanTopic("Note").publish();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    movingForward = (m_speed.getAsDouble() > 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.spinDirect(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The intake stops spinning when the command ends.
    m_IntakeSubsystem.stopIntake();
    if (ShooterSpeedSub.getAsDouble() > 20){
      NotePub.set(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // If in feeding mode, don't end
    if(m_isFeeding.getAsBoolean()){
      return false;
    
    // If NOT in feeding mode, end if moving forward and a note is detected.
    } else {
      if (movingForward && m_IntakeSubsystem.noteDetected()) {
        m_IntakeSubsystem.setHasNote(true);
        return true;

      } else {
        return false;
      }
    }
  }
}

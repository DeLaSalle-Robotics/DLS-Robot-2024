package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class ClimberWinch extends Command {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final BooleanSupplier m_movingUp;

  /**
   * 
   * @param subsystem ClimberSubsystem
   * @param movingUp
   */
  public ClimberWinch(ClimberSubsystem subsystem, BooleanSupplier movingUp) {
    m_ClimberSubsystem = subsystem;
    m_movingUp = movingUp;
    addRequirements(m_ClimberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If the winch exceeds its current limit, reset the encoder position
    if(m_ClimberSubsystem.getWinchCurrent() >= Constants.Climber.kWinchCurrentLimit){
      m_ClimberSubsystem.setWinchLowerLimit();
    }

    // Changes the direction that the motors will move in
    int direction = m_movingUp.getAsBoolean()? 1:-1;

    // Spin winch motor
    // Raw power because the PID controller is not tuned for heavy loads
    m_ClimberSubsystem.spinWinch(direction * Constants.Climber.kWinchPowerInWinchMode);

    // The extender motor should not be moving
    m_ClimberSubsystem.spinExtender(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_ClimberSubsystem.spinExtender(0.0);
    m_ClimberSubsystem.spinWinch(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



















//cndkshfkushlfhgeshjjo;/laiwfgukbifdkgho;gudhrishvkljhfmhsiozhgfkjekahflkgksehkgkfjjzklrsyfkjs,e.ayi fwhlefgui sezufiol hnevvfubs;aiohfigeysifluhi;h
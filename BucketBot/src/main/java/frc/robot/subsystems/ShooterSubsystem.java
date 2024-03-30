package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.Shooter;


public class ShooterSubsystem extends SubsystemBase {

  // Declare Falcon controllers
  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;

  final VelocityVoltage m_velocity = new VelocityVoltage(0);
  

  // ShooterSubsystem constructor
  public ShooterSubsystem() {
    super();

    m_shooterMotor1 = new TalonFX(Constants.Shooter.kShooterMotorID1);
    m_shooterMotor2 = new TalonFX(Constants.Shooter.kShooterMotorID2);

    m_shooterMotor1.getConfigurator().apply(new TalonFXConfiguration()); // Resets the motor configurations
    m_shooterMotor2.getConfigurator().apply(new TalonFXConfiguration());

    m_shooterMotor1.setInverted(true);

    // Allows editing shooter speeds for testing purposes
    SmartDashboard.putNumber("Amp Speed", Constants.Shooter.kAmpSpeed);
    SmartDashboard.putNumber("Speaker Speed", Constants.Shooter.kSpeakerSpeedRPS);

    // Velocity control for shooter motor using FF and P controller
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.Shooter.kShooterFFS; // Voltage Required to move the motors at all. 
    slot0Configs.kV = Constants.Shooter.kShooterFFV; // Same as kF - V per rot/s <-- Calculated from applied voltage/ spin velocity
    slot0Configs.kP = Constants.Shooter.kShooterKP; // Proportional factor needs tuning
    slot0Configs.kI = Constants.Shooter.kShooterKI;  // Integral factor useful if not getting to set point
    slot0Configs.kD = Constants.Shooter.kShooterKD; // derivative factor to prevent overshoot
    // Put these control values into Slot0 in the talonFx controller- with a 50 ms overflow limit.
    m_shooterMotor1.getConfigurator().apply(slot0Configs, 0.050);
    m_shooterMotor2.getConfigurator().apply(slot0Configs, 0.050);
  }


  /**
   * Spin the motors at the given power
   * @param power Double between -1.0 and 1.0
   */
  public void spin(double power){

    SmartDashboard.putNumber("Shooter Commanded Power", power);
    
    // Real motors
    m_shooterMotor1.set(power);
    m_shooterMotor2.set(power);
  }


/**
 * Spin the shooter at the given speed with PID control.
 * @param speed Speed in rotations per second
 */
public void spinAt(double speed){
  m_velocity.Slot = 0;
  m_shooterMotor1.setControl(m_velocity.withVelocity(speed));
  m_shooterMotor2.setControl(m_velocity.withVelocity(speed));
}



/**
 * <b>Auto Shooter Command</b>
 * <p>We start 2 commands at the same time: one to spin up the shooter and one to (eventually) run the intake. 
 * The intake command will wait until the shooter is up to speed, then it will run the intake for 1 second.
 * <p>Once the intake command is done, the shooter command will end. 
 * This is because the intake sequence is the "deadline" command,
 * so when the intake command ends, the shooter command is interrupted
 * @param intake IntakeSubsystem
 * @return A command, with functionality as described above.
 */
public Command autoShooter(IntakeSubsystem intake){
    return Commands.deadline(  

        // Command A (deadline): Once shooter is up to speed, run the intake for 1 second
        new WaitUntilCommand(
          () -> this.atSpeed()

        ).andThen(
          new Intake(
            intake, 
            () -> 0.5, 
            () -> true
          ).withTimeout(1.0)
        ),

        // Command B: Spin shooter until intake command is complete
        new Shooter(this)
    );
}


/**
 * Check if the shooter is at speed.
 * <p>Used specifically for {@link #autoShooter}.
 * @return True if the shooter is at speed
 */
public boolean atSpeed() {
  // Store the speed of each motor
  double speed1 = m_shooterMotor1.getVelocity().getValueAsDouble();
  double speed2 = m_shooterMotor2.getVelocity().getValueAsDouble();

  // Check if each motor is within the speed threshold
  boolean check1 = Math.abs(speed1 - Constants.Shooter.kSpeakerSpeedRPS) < 5;
  boolean check2 = Math.abs(speed2 - Constants.Shooter.kSpeakerSpeedRPS) < 5;

  // Return true if at least one of the motors is within the speed threshold 
  return check1 || check2;
}


  // Unused
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  
  // Unused
  // Query some boolean state, such as a digital sensor.
  public boolean exampleCondition() {
    return false;
  }


  // Called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity 1 (RPS)", m_shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity 2 (RPS)", m_shooterMotor2.getVelocity().getValueAsDouble());
  }


  // Called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}

}
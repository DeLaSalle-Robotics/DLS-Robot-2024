package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LED.LED_State;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class LED extends Command {
  public enum LED_State {
    WAITING,
    NO_NOTE,
    HAVE_NOTE,
    IN_ZONE,
    ON_TARGET
  }
  // Command parameters
  private final LEDSubsystem m_LEDSubsystem;
  private LED_State currentState;
  
   BooleanSubscriber InZoneSub;
    BooleanSubscriber OnTargetSub;
    BooleanSubscriber NoteSub;
    BooleanSubscriber RestingSub;

  /**
   * Control the LEDs based on what other subsystems are doing
   * @param ledSubsystem LEDSubsystem
   * @param intakeSubsystem IntakeSubsystem
   */
  public LED(LEDSubsystem ledSubsystem) {
    m_LEDSubsystem = ledSubsystem;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    InZoneSub = table.getBooleanTopic("InZone").subscribe(false);
    NoteSub = table.getBooleanTopic("Note").subscribe(false);
    OnTargetSub = table.getBooleanTopic("OnTarget").subscribe(false);
    RestingSub = table.getBooleanTopic("Resting").subscribe(false);


    addRequirements(m_LEDSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = LED_State.WAITING;

  }

  /**
   * Control the LEDs based on the state of the other subsystems
   * <p>Blue: Intake has a note
   * <p>Off: Otherwise
   */
  @Override
  public void execute() {
    /*if(m_IntakeSubsystem.hasNote()){
      m_LEDSubsystem.set(Color.kBlue);
    } else {
      m_LEDSubsystem.rainbow();
//      m_LEDSubsystem.set(Color.kRed);
    }*/ 
    //String led_state = SmartDashboard.getString("LED State","None");

    //Reaching out to other subsystems to identify the state for LED display

    if (NoteSub.getAsBoolean()) {this.currentState = LED_State.HAVE_NOTE;}else {currentState = LED_State.NO_NOTE;}
    if (InZoneSub.getAsBoolean() //&& NoteSub.getAsBoolean()
        ) {this.currentState = LED_State.IN_ZONE;}
    if (OnTargetSub.getAsBoolean() //&& NoteSub.getAsBoolean()
        ) {this.currentState = LED_State.ON_TARGET;}
    if (RestingSub.getAsBoolean()) {this.currentState = LED_State.WAITING;}

// Key is to have a series of methods that define the states within subsystems
    switch (currentState) {
        case HAVE_NOTE -> m_LEDSubsystem.set(Color.kBlue);
        case NO_NOTE -> { m_LEDSubsystem.off();}
        case IN_ZONE-> m_LEDSubsystem.set(Color.kRed);
        case WAITING-> m_LEDSubsystem.rainbow();
        case ON_TARGET -> m_LEDSubsystem.set(Color.kGreen);
        default -> m_LEDSubsystem.off();
      }
      SmartDashboard.putString("LED State", currentState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Turn off the LEDs
    m_LEDSubsystem.rainbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

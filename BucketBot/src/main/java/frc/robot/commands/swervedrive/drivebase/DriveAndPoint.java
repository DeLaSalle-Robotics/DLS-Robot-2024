// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class DriveAndPoint extends Command
{

  private final SwerveSubsystem m_swerve;
  private final DoubleSupplier vX, vY, heading;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. 
   *
   * @param m_swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle in degrees
   */
  public DriveAndPoint(SwerveSubsystem m_swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier heading)
  {
    this.m_swerve = m_swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(m_swerve);
  }

  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = m_swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                         new Rotation2d(heading.getAsDouble()/180.0 * Math.PI));

    // Make the robot move
    m_swerve.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
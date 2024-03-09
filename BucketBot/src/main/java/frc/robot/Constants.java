package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
        
        // Shooter constants
        public static class Shooter {

                // CAN IDs of the shooter motors
                public static final int kShooterMotorID1 = 10;
                public static final int kShooterMotorID2 = 12;

                // Maximum voltage of the shooter motors
                public static final double kShooterMaxVolts = 5.0;
        }

        public static class Climber {

                // CAN IDs of the climber motors
                public static final int kExtenderMotorID = 2;
                public static final int kClimberMotorID = 7;

                public static final double kExtenderMotorVelocityRPM = 900.0;
                public static final double kClimberMotorVelocityRPM = 1440.0;

                public static final double kClimberFeedForwardUp = -0.2; // -0.244
                public static final double kClimberFeedForwardDown = 0.244;
                
                public static final double kExtenderFeedForwardUp = 0.06;
                public static final double kExtenderFeedForwardDown = -0.1;

                public static final double kPExtenderDown = 0.00001;
                public static final double kPClimberDown = 0.00002;

                public static final double kPExtenderUp = 0.00001;
                public static final double kPClimberUp = 0.00001;

                // Endpoints for extender motor
                public static final double kExtenderEndpointUp = -44.0;
                public static final double kExtenderEndpointDown = -119.0;
                public static final float kExtenderDistance = 75.0f;

                // Endpoints for climber motor
                public static final double kClimberEndpointUp = 2.1;
                public static final double kClimberEndpointDown = 122.0;
                public static final float kClimberDistance = 119.9f;
        }
        


        // Intake constants
        public static class Intake {
                
                // CAN ID of the intake motor
                public static final int kIntakeMotorID = 75;

                // DIO pin that the limit switch is connected to
                public static final int kIntakeLimitSwitchID = 0;
        } 



        // Swerve module constants
        public static final class ModuleConstants {

                // Diameter of the wheels
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

                // Gear ratio of swerve module motors
                public static final double kDriveMotorGearRatio = 1 / 5.8462;
                public static final double kTurningMotorGearRatio = 1 / 18.0;

                // Unit conversions for drive encoder
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

                // Unit conversions for turning encoder
                public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
                public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

                public static final double kPTurning = 0.5;
        }



        // Driving constants
        public static final class DriveConstants {

                /**Distance between left and right wheels.*/
                public static final double kTrackWidth = Units.inchesToMeters(21);

                /**Distance between front and back wheels.*/
                public static final double kWheelBase = Units.inchesToMeters(25.5);

                /**Kinematics for each swerve module.*/
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
                );

                // CAN IDs of each module's drive motor
                public static final int kFrontLeftDriveMotorPort = 8;
                public static final int kBackLeftDriveMotorPort = 2;
                public static final int kFrontRightDriveMotorPort = 6;
                public static final int kBackRightDriveMotorPort = 4;

                // CAN IDs of each module's turning motor
                public static final int kFrontLeftTurningMotorPort = 7;
                public static final int kBackLeftTurningMotorPort = 1;
                public static final int kFrontRightTurningMotorPort = 5;
                public static final int kBackRightTurningMotorPort = 3;

                // Whether or not each module's turning encoder is reversed
                public static final boolean kFrontLeftTurningEncoderReversed = true;
                public static final boolean kBackLeftTurningEncoderReversed = true;
                public static final boolean kFrontRightTurningEncoderReversed = true;
                public static final boolean kBackRightTurningEncoderReversed = true;

                // Whether or not each module's drive encoder is reversed
                public static final boolean kFrontLeftDriveEncoderReversed = true;
                public static final boolean kBackLeftDriveEncoderReversed = true;
                public static final boolean kFrontRightDriveEncoderReversed = false;
                public static final boolean kBackRightDriveEncoderReversed = false;

                // CAN IDs of each module's absolute encoder
                public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
                public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
                public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
                public static final int kBackRightDriveAbsoluteEncoderPort = 3;

                // Whether or not each module's absolute encoder is reversed
                public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                // Offset of each module's absolute encoder, in radians
                public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
                public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
                public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
                public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

                /**Maximum PHYSICAL speed of the drive motors.*/
                public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
                /**Maximum PHYSICAL angular speed of the turning motors.*/
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                /**Maximum tele-op driving speed of the drive motors.*/
                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
                /**Maximum tele-op acceleration of the drive motors.*/
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;

                /**Maximum tele-op angular speed of the turning motors.*/
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
                /**Maximum tele-op angular acceleration of the turning motors.*/
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }


        
        // Autonomous constants
        public static final class AutoConstants {

                /**Maximum AUTO driving speed of the drive motors.*/
                public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
                /**Maximum AUTO acceleration of the drive motors.*/
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;

                /**Maximum AUTO angular speed of the turning motors.*/
                public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
                /**Maximum AUTO angular acceleration of the turning motors.*/
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

                public static final double kPXController = 1.5;
                public static final double kPYController = 1.5;
                public static final double kPThetaController = 3;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared
                );
        }


        public static final class OIConstants {

                /**Port of the current controller used for driving.*/
                public static final int kDriverControllerPort = 0;

                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                public static final double kDeadband = 0.05;
        }
}

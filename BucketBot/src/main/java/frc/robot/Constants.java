package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public final class Constants {
        
        // Shooter constants
        public static final class Shooter {

                // CAN IDs of the shooter motors
                public static final int kShooterMotorID1 = 13;
                public static final int kShooterMotorID2 = 14;

                // Maximum voltage of the shooter motors
                public static final double kShooterMaxVolts = 5.0;
        }
        


        // Intake constants
        public static final class Intake {
                
                // CAN ID of the intake motor
                public static final int kIntakeMotorID = 15;

                // DIO pin that the limit switch is connected to
                public static final int kIntakeLimitSwitchID = 0;

                // PID Information for the intake
                public static final double kPIntake = 0.01;
        } 



        public static final class OIConstants {

                /**Port of the current controller used for driving.*/
                public static final int kDriverControllerPort = 0;

                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                public static final double kDeadband = 0.1;
        }


        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

        public static final class Auton
        {

                public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
                public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

                public static final double MAX_ACCELERATION = 2;

                public static final String PathFileName = "Experimental"; // Use this to switch which pathfinder file to run in auto
        }

        public static final class Drivebase
        {

                // Hold time on motor brakes when disabled
                public static final double WHEEL_LOCK_TIME = 10; // seconds

                public static final double maxSpeed = 10;
        }

        public static final class OperatorConstants
        {

                // Joystick Deadband
                public static final double LEFT_X_DEADBAND  = 0.1;
                public static final double LEFT_Y_DEADBAND  = 0.1;
                public static final double RIGHT_X_DEADBAND = 0.1;
                public static final double TURN_CONSTANT    = 6;
        }

        public static final class VisionConstants
        {
                // Simulated camera properties, based mostly on real camera

                public static final int resWidth = 960;
                public static final int resHeight = 720;
                public static final double fovDiagDegrees = 74.8;

                public static final double calibErrorPx = 0.146;
                public static final double calibErrorStdDev = 0.0486;

                public static final double fps = 45.0;
                public static final double avgLatencyMs = 310.0;

                // For position (relative to the robot):
                // X: Forward/Backward
                // Y: Left/Right
                // Z: Up/Down
                public static final Translation3d cameraPosition = new Translation3d(0.0, 0.0, 0.5);
                public static final Rotation3d cameraRotation = new Rotation3d(0.0, 0.0, 0.0);
        }


        public static final class Climber {

                // CAN IDs of the climber motors
                public static final int kExtenderMotorID = 16;
                public static final int kClimberMotorID = 17;

                // DIO pin of the limit switch
                public static final int kLimitSwitchID = 2;

                // Target velocity of each climber motor
                public static final double kExtenderMotorVelocityRPM = 900.0;
                public static final double kClimberMotorVelocityRPM = 1440.0;

                // Feed forward of the extender motor
                public static final double kExtenderFeedForwardUp = 0.06;
                public static final double kExtenderFeedForwardDown = -0.1;

                // Feed forward of the climb motor
                public static final double kClimberFeedForwardUp = -0.2; // -0.244
                public static final double kClimberFeedForwardDown = 0.244;

                // kP of the extender motor
                public static final double kPExtenderDown = 0.00001;
                public static final double kPExtenderUp = 0.00001;

                // kP of the climb  motor
                public static final double kPClimberUp = 0.00001;
                public static final double kPClimberDown = 0.00002;

                // Endpoints for extender motor
                public static final double kExtenderEndpointUp = -44.0;
                public static final double kExtenderEndpointDown = -119.0;
                public static final float kExtenderDistanceCm = 42.8625f;

                // Endpoints for climber motor
                public static final double kClimberEndpointUp = 2.1;
                public static final double kClimberEndpointDown = 122.0;
                public static final float kClimberDistanceCm = 119.9f; // Incorrect

                // Conversion factors
                public static final double kExtenderCmPerRotation = 0.59;
                public static final double kClimberCmPerRotation = 0.392;


                public static final double kMotorOffset = 0.2;
        }
}

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
        public static class Shooter {

                // CAN IDs of the shooter motors
                public static final int kShooterMotorID1 = 13;
                public static final int kShooterMotorID2 = 14;

                // Maximum voltage of the shooter motors
                public static final double kShooterMaxVolts = 5.0;
        }
        


        // Intake constants
        public static class Intake {
                
                // CAN ID of the intake motor
                public static final int kIntakeMotorID = 15;

                // DIO pin that the limit switch is connected to
                public static final int kIntakeLimitSwitchID = 0;
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

        public static class OperatorConstants
        {

                // Joystick Deadband
                public static final double LEFT_X_DEADBAND  = 0.1;
                public static final double LEFT_Y_DEADBAND  = 0.1;
                public static final double RIGHT_X_DEADBAND = 0.1;
                public static final double TURN_CONSTANT    = 6;
        }

        public static class VisionConstants
        {
                // Simulated camera properties, based mostly on real camera

                public static final int resWidth = 960;
                public static final int resHeight = 720;
                public static final double fovDiagDegrees = 74.8;

                public static final double calibErrorPx = 0.146;
                public static final double calibErrorStdDev = 0.0486;

                public static final double fps = 45.0;
                public static final double avgLatencyMs = 310.0;

                public static final Translation3d cameraPosition = new Translation3d(0.0, 0.0, 0.5);
                public static final Rotation3d cameraRotation = new Rotation3d(0.0, 0.0, 0.0);
        }
}

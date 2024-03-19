package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
                public static final double kPIntake = 0.005;
        } 


        // Operator constants
        public static final class OperatorConstants {

                /**Port of the current controller used for driving.*/
                public static final int kDriverControllerPort = 0;
 
                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                // Controller deadband
                public static final double kDeadband = 0.1;

                // Some YAGSL stuff
                public static final double kLeftXDeadband = 0.1;
                public static final double kLeftYDeadband = 0.1;
                public static final double kRightXDeadband = 0.1;
                public static final double kTurnConstant = 6;
        }


        // Autonomous Constants
        public static final class AutoConstants
        {
                // All of this is YAGSL stuff
                public static final PIDFConfig kTranslationPID = new PIDFConfig(0.7, 0, 0);
                public static final PIDFConfig kAngleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

                public static final double kMaxAcceleration = 2;

                public static final String kPathFileName = "Experimental"; // Use this to switch which pathplanner file to run in auto
        }

        public static final class Drivebase
        {

                // Hold time on motor brakes when disabled
                public static final double kWheelLockTime = 10; // seconds

                public static final double kMaxSpeed = 10;

                // More YAGSL stuff
                public static final double kRobotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
                public static final Matter kChassis = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass);
                public static final double kLoopTime = 0.13; //s, 20ms + 110ms sprk max velocity lag
        }

        public static final class VisionConstants
        {
                // Simulated camera properties, based mostly on real camera
                public static final int kResWidth = 960;
                public static final int kResHeight = 720;
                public static final double kFovDiagDegrees = 74.8;

                public static final double kCalibErrorPx = 0.146;
                public static final double kCalibErrorStdDev = 0.0486;

                public static final double kFps = 45.0;
                public static final double kAvgLatencyMs = 310.0;

                // For position (relative to the robot):
                // X: Forward/Backward
                // Y: Left/Right
                // Z: Up/Down
                public static final Translation3d kCameraPosition = new Translation3d(0.0, 0.0, 0.5);
                public static final Rotation3d kCameraRotation = new Rotation3d(0.0, 0.0, 0.0);
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

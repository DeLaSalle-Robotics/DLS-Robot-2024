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

                // Set speeds for shooting at the amp and speaker, respectively
                public static final double kAmpSpeed = 0.3;
                public static final double kSpeakerSpeedRPS = 30;

                // Reverse shooter power
                public static final double kShooterReversePower = 0.1;

                // Feedforward and PID for shooter motor
                public static final double kShooterFFS = 0.05;
                public static final double kShooterFFV = 0.016;
                public static final double kShooterKP = 0.01;
                public static final double kShooterKI = 0.0;
                public static final double kShooterKD = 0.0;
        }
        


        // Intake constants
        public static final class Intake {
                
                // CAN ID of the intake motor
                public static final int kIntakeMotorID = 15;

                // DIO pin that the limit switch is connected to
                public static final int kIntakeLimitSwitchID = 0;

                // PID Information for the intake
                public static final double kPIntake = 0.005;

                // Intake target speeds
                public static final double kIntakeTargetSpeed = 0.75;
                public static final double kIntakeFeederSpeed = 0.3;
        } 


        // Operator constants
        public static final class OperatorConstants {

                /**Port of the current controller used for driving.*/
                public static final int kDriverControllerPort = 0;

                // Flight joystick ports, in case they are ever used
                // For purposes of organization, the joystick with the "Left" label is considered L, and the other R
                public static final int kFlightJoystickPortR = 2;
                public static final int kFlightJoystickPortL = 1;
 
                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                // Trigger deadband
                public static final double kDeadband = 0.1;

                // Deadband
                public static final double kLeftXDeadband = 0.1;
                public static final double kLeftYDeadband = 0.1;
                public static final double kRightXDeadband = 0.1;
                public static final double kRightYDeadband = 0.1;

                
                public static final double kTurnConstant = 6;
        }


        // Autonomous Constants
        public static final class AutoConstants
        {
                // All of this is YAGSL stuff
                public static final PIDFConfig kTranslationPID = new PIDFConfig(0.7, 0, 0);
                public static final PIDFConfig kAngleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

                public static final double kMaxAcceleration = 2;

                public static final String kPathFileName = "Backup"; // Use this to switch which pathplanner file to run in auto
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
                public static final int kWinchMotorID = 17;

                // DIO pin of the limit switch
                public static final int kLimitSwitchID = 2;

                // Target velocity of each climber motor
                public static final double kClimberVelocityCmPerSec = 3.0;

                // Winch power in winch mode!!!
                public static final double kWinchPowerInWinchMode = 0.0;

                // Current limit of the winch motor during winch mode
                public static final double kWinchCurrentLimit = 1000.0;

                // Old feed forward of the extender motor
                public static final double kExtenderFeedForwardUp = 0.06;
                public static final double kExtenderFeedForwardDown = -0.1;

                // Old feed forward of the winch motor
                public static final double kWinchFeedForwardUp = -0.2; // -0.244
                public static final double kWinchFeedForwardDown = 0.244;

                // Old kP of the extender motor
                public static final double kPExtenderDown = 0.00001;
                public static final double kPExtenderUp = 0.00001;

                // Old kP of the winch motor
                public static final double kPWinchUp = 0.00001;
                public static final double kPWinchDown = 0.00002;

                // Feedforward and PID for extender motor
                public static final double kExtenderFFS = 0.0;
                public static final double kExtenderFFV = 0.0;
                public static final double kExtenderFFG = 0.0;
                public static final double kExtenderKP = 0.0;
                public static final double kExtenderKI = 0.0;
                public static final double kExtenderKD = 0.0;

                // Feedforward and PID for winch motor
                public static final double kWinchFFS = 0.0;
                public static final double kWinchFFV = 0.0;
                public static final double kWinchFFG = 0.0;
                public static final double kWinchKP = 0.0;
                public static final double kWinchKI = 0.0;
                public static final double kWinchKD = 0.0;


                // Power limits for climber motors
                public static final double kExtenderMinPower = -1.0;
                public static final double kExtenderMaxPower = 1.0;
                public static final double kWinchMinPower = -1.0;
                public static final double kWinchMaxPower = 1.0;

                // Endpoints for extender motor
                public static final double kExtenderEndpointUp = 100.0;
                public static final double kExtenderEndpointDown = 0.0;
                public static final float kExtenderDistanceCm = 42.8625f;

                // Endpoints for winch motor
                public static final double kWinchEndpointUp = 100.0;
                public static final double kWinchEndpointDown = -10.0;
                public static final float kWinchDistanceCm = 119.9f; // Incorrect

                // Conversion factors
                public static final double kExtenderCmPerRotation = 0.59;
                public static final double kWinchCmPerRotation = 0.392;

                public static final double kMotorOffset = 0.2;
        }
}


















// me when I constantly have a constant constant in constants with a constantly constanting constant that is always constant
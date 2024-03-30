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
                public static final double kSpeakerSpeedRPS = 45;

                // Reverse shooter power
                public static final double kShooterReversePower = 0.1;

                // Feedforward and PID for shooter motor
                public static final double kShooterFFS = 0.48; // volts
                public static final double kShooterFFV = 0.11; //0.11 Volts per RPS
                public static final double kShooterKP = 0.15; 
                public static final double kShooterKI = 0.0;
                public static final double kShooterKD = 0.0;
        }
        


        // Intake constants
        public static final class Intake {
                
                // CAN ID of the intake motor
                public static final int kIntakeMotorID = 15;

                // Intake target speeds
                public static final double kIntakePower = 0.5;
                public static final double kIntakeReversePower = 0.2;
                public static final double kIntakeFeederPower = 0.35;
        } 


        // Operator constants
        public static final class OperatorConstants {

                /**Port of the current controller used for driving.*/
                public static final int kDriverControllerPort = 0;

                // Flight joystick ports, in case they are ever used
                // For purposes of organization, the joystick with the "Left" label is considered L, and the other R
                public static final int kFlightJoystickPortR = 2;
                public static final int kFlightJoystickPortL = 1;

                // Port of the controller used for testing
                public static final int kTestControllerPort = 3;
 
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
                public static final double kExtenderTargetVelocity = 3.0; // cm/s
                public static final double kWinchTargetVelocity = 3.0; // cm/s

                // Simple climber power constants
                public static final double kExtenderExtendPower = 0.1;
                public static final double kExtenderRetractPower = -0.2;
                public static final double kWinchExtendPower = 0.01;
                public static final double kWinchRetractPower = -0.15;

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

                // Endpoints for extender motor
                public static final double kExtenderEndpointUp = 44.0; //cm
                public static final double kExtenderEndpointDown = 0.0;

                // Endpoints for winch motor
                public static final double kWinchEndpointUp = 0;
                public static final double kWinchEndpointDown = -91.0; //cm

                // Conversion factors
                public static final double kExtenderCmPerRotation = 0.59;
                public static final double kWinchCmPerRotation = 0.392;

        }
}


















// me when I constantly have a constant constant in constants with a constantly constanting constant that is always constant
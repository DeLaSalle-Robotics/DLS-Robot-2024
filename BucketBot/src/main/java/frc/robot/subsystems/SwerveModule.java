package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {

    // Declare motors
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    // Declare encoders
    private final RelativeEncoder turningEncoder;

    // Declare absolute encoders
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    // Declare PID Controllers
    private final PIDController turningPidController;


    /**
     * Creates a new swerve module. This should be created using constants.
     * @param driveMotorId k...DriveMotorPort
     * @param turningMotorId k...TurningMotorPort
     * @param driveMotorReversed k...DriveEncoderReversed
     * @param turningMotorReversed k...TurningEncoderReversed
     * @param absoluteEncoderId k...DriveAbsoluteEncoderPort
     * @param absoluteEncoderOffset k...DriveAbsoluteEncoderOffsetRad
     * @param absoluteEncoderReversed k...DriveAbsoluteEncoderReversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        // Declare absolute encoders
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // Categorize the motors of this module
        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        // Set the inversion state of both motors
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // Declare the turning motor encoder
        turningEncoder = turningMotor.getEncoder();

        // Set measurement factors of the turning encoder
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // Create a PID controller for the turning motor
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset the encoders to zero
        resetEncoders();
    }


    /**
     * Gets the position of a drive motor from a swerve module.
     * @return The position of the drive motor, in meters.
     */
    public double getDrivePosition() {

        // Get the drive motor's position as a status signal and obtain its value
        StatusSignal<Double> ss = driveMotor.getPosition();
        double drivePosition = ss.refresh().getValue();

        // Convert units from rotations to meters
        return drivePosition * ModuleConstants.kDriveEncoderRot2Meter;
    }


    /**
     * Gets the state of a swerve module, including position and rotation.
     * @return The state of the given swerve module.
     */
    public SwerveModulePosition getPosition() {
        Rotation2d rot = new Rotation2d(ModuleConstants.kTurningEncoderRot2Rad);
        SwerveModulePosition position = new SwerveModulePosition(this.getDrivePosition(), rot);
        return position;
    }


    /**
     * Gets the position of the turning motor of a swerve module.
     * @return The position of the turning motor's encoder, in rotations.
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }


    /**
     * Gets the current velocity of the drive motor of a swerve module.
     * @return The current velocity of the drive motor, in meters per second.
     */
    public double getDriveVelocity() {

        // Get the velocity of the drive motor as a status signal and obtain its value
        StatusSignal<Double> ss = driveMotor.getVelocity();
        double drivePosition = ss.refresh().getValue();

        // Convert units from rotations per second to meters per second
        return drivePosition * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }


    /**
     * Gets the current velocity of the turning motor of a swerve module.
     * @return The current velocity of the turning motor, in rotations per minute.
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }


    /**
     * Gets the default position of a turning motor's absolute encoder.
     * Used in SwerveModule.resetEncoders() to reset the absolute encoders since they don't automatically reset.
     * @return The default position of the absolute encoder, in radians.
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }


    /**
     * Resets all encoders of a swerve module to their default values, including the absolute encoder.
     */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }


    /**
     * Gets the state of a swerve module as a SwerveModuleState object, which stores both the drive velocity and the turning position.
     * @return The state of the given swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }


    /**
     * Sets the state of a swerve module. This can include changing the speed or rotation of the given module.
     * @param state The desired state of a swerve module.
     */
    public void setDesiredState(SwerveModuleState state) {

        // If the desired state's speed is close enough to zero, stop the motors and end the method.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the turning of the swerve module
        // For example, instead of turning 350 degrees to the right, it would turn 10 degrees to the left
        state = SwerveModuleState.optimize(state, getState().angle);

        // Set the speed of the drive motor and the position of the turning motor
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        // Post the swerve state to SmartDashboard
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }


    /**
     * Stops both motors in a swerve module.
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    //  hi
}

/*
 * Example subsystem
 * Copy this to create another subsystem
 */

package frc.robot.subsystems;

import frc.robot.Robot;

import org.photonvision.PhotonCamera;
// import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 6 inches up, 6 inches to the side for testing

public class VisionSubsystem extends SubsystemBase {

  // Vision simulator
  private final VisionSystemSim visionSim = new VisionSystemSim("john");

  // Create the simulated camera
  private final PhotonCamera camera = new PhotonCamera("jane");
  private final PhotonCameraSim vs_camera;

  // Simulation camera properties
  private final SimCameraProperties vs_camProperties = new SimCameraProperties();

  // April tag model and field layout
  // private final TargetModel vs_targetModel = TargetModel.kAprilTag36h11;
  private final AprilTagFieldLayout vs_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Swerve subsystem to be passed through the constructor
  private final SwerveSubsystem m_swerveSubsystem;



  // VisionSubsystem constructor
  public VisionSubsystem(SwerveSubsystem swerve) {

    // Define swerve subsystem
    m_swerveSubsystem = swerve;

    // Add april tags from the april tag layout
    visionSim.addAprilTags(vs_layout);

    // Set resolution, FOV, calibration errors, FPS, and average latency
    vs_camProperties.setCalibration(960, 720, Rotation2d.fromDegrees(74.8));
    vs_camProperties.setCalibError(0.146, 0.0486);
    vs_camProperties.setFPS(45);
    vs_camProperties.setAvgLatencyMs(310);

    // Add the properties to the simulated camera
    vs_camera = new PhotonCameraSim(camera, vs_camProperties);

    // Allow the simulated camera to draw a wireframe visualization of the field to the camera streams
    // This is very resource intensive and reccomended to be left off
    vs_camera.enableDrawWireframe(true);

    // Position and rotation of the camera relative to the robot pose
    Translation3d vs_camPosition = new Translation3d(0.0, 0.0, 0.5);
    Rotation3d vs_camRotation = new Rotation3d(0.0, 0.0, 0.0); // Note that this is in radians - use Math.toRadians to enter degree values
    Transform3d vs_camTranslation = new Transform3d(vs_camPosition, vs_camRotation);

    // Add the camera to the vision simulator
    visionSim.addCamera(vs_camera, vs_camTranslation);
  }


  public PhotonPipelineResult getLatestResult(){
    return null;
  }



  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {

  }


  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {
    visionSim.update(m_swerveSubsystem.getPose());
  }



  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

}

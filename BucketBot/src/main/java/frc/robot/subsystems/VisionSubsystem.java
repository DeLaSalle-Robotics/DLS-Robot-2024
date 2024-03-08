/*
 * Example subsystem
 * Copy this to create another subsystem
 */

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 6 inches up, 6 inches to the side for testing

public class VisionSubsystem extends SubsystemBase {

  /** Vision simulator */
  private final VisionSystemSim visionSim = new VisionSystemSim("john");

  /** Camera simulator */
  private final PhotonCamera camera = new PhotonCamera("jane");
  private final PhotonCameraSim vs_camera;

  // Simulation camera properties
  private final SimCameraProperties vs_camProperties = new SimCameraProperties();

  // April tag model and field layout
  // private final TargetModel vs_targetModel = TargetModel.kAprilTag36h11;
  private final AprilTagFieldLayout vs_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Swerve subsystem to be passed through the constructor
  private final SwerveSubsystem m_swerveSubsystem;

  // List of camera targets, see periodic/simulationPeriodic
  private List<PhotonTrackedTarget> targetList;

  // Position and orientation of the camera, robot-relative
  private final Transform3d vs_camTranslation;



  // VisionSubsystem constructor
  public VisionSubsystem(SwerveSubsystem swerve) {

    // Define swerve subsystem, we need methods from it for vision to work
    m_swerveSubsystem = swerve;

    // Add april tags from the april tag layout
    visionSim.addAprilTags(vs_layout);

    // Set resolution and FOV calibration errors, FPS, and average latency
    vs_camProperties.setCalibration(
      VisionConstants.resWidth,
      VisionConstants.resHeight, 
      Rotation2d.fromDegrees(VisionConstants.fovDiagDegrees)
    );

    // Set calibration errors
    vs_camProperties.setCalibError(
      VisionConstants.calibErrorPx, 
      VisionConstants.calibErrorStdDev
    );

    // Set FPS and average latency
    vs_camProperties.setFPS(VisionConstants.fps);
    vs_camProperties.setAvgLatencyMs(VisionConstants.avgLatencyMs);

    // Add the properties to the simulated camera
    vs_camera = new PhotonCameraSim(camera, vs_camProperties);

    // Allow the simulated camera to draw a wireframe visualization of the field to the camera streams
    // This is very resource intensive and reccomended to be left off
    vs_camera.enableDrawWireframe(false);

    // Position and rotation of the camera relative to the robot pose
    Translation3d vs_camPosition = VisionConstants.cameraPosition;
    Rotation3d vs_camRotation = VisionConstants.cameraRotation; // Note that this is in radians - use Math.toRadians to enter degree values
    vs_camTranslation = new Transform3d(vs_camPosition, vs_camRotation);

    // Add the camera to the vision simulator
    visionSim.addCamera(vs_camera, vs_camTranslation);
  }


  /**
   * <b>This is unfinished nor is it tested and probably should not be used.</b>
   * <p>Re-orient the robot based on april tags.
   */
  @Deprecated
  public void orientRobot(){

    // Define latest camera result
    PhotonPipelineResult result = camera.getLatestResult();

    // If a target does not exist, cancel the operation
    if (result.hasTargets()) {

      // Define the best target
      PhotonTrackedTarget target = result.getBestTarget();

      // Estimate robot pose
      Pose3d estRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        vs_layout.getTagPose(target.getFiducialId()).get(),
        vs_camTranslation
      );

      // Set robot pose
      m_swerveSubsystem.resetOdometry(estRobotPose.toPose2d());
    }

  }


  public void watchAprilTag(int aprilTagID){
    
    // Find the specified april tag
    Pose3d tagPose = vs_layout.getTagPose(aprilTagID).get();

    // Get the rotation to the april tag
    Rotation2d rotationToTag = PhotonUtils.getYawToPose(m_swerveSubsystem.getPose(), tagPose.toPose2d());

    // Set heading to the specified tag
    m_swerveSubsystem.driveCommand(
      () -> 0.0, 
      () -> 0.0, 
      () -> rotationToTag.getRotations(),
      () -> false
    );

  }

  


 public PhotonCamera getPhotonCamera(){
    return camera;
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

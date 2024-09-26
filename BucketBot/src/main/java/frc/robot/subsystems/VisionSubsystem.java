package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// 6 inches up, 6 inches to the side for testing

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;

  private final SwerveSubsystem swerveSubsystem;

  public PhotonPoseEstimator photonPoseEstimator;

  private AprilTagFieldLayout aprilTagFieldLayout;
  
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(10));

  private final Field2d field2d = new Field2d();

  private Transform3d robotToCam;

  private double previousPipelineTimeStamp = 0;

  private final SwerveDrivePoseEstimator poseEstimator;

// VisionSubsystem constructor
public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
  // Load the camera
  camera = new PhotonCamera("Logitech_Webcam_C930e");
// Adding the swerve subsystem to this class
  this.swerveSubsystem = swerveSubsystem;
  //Load field layout
  AprilTagFieldLayout layout;
  layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  layout.setOrigin( OriginPosition.kBlueAllianceWallRightSide); 
  //Camera Position Relative to center of robot.
  robotToCam = new Transform3d(Constants.VisionConstants.kCameraPosition, Constants.VisionConstants.kCameraRotation);
  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  
  poseEstimator = new SwerveDrivePoseEstimator(swerveSubsystem.getKinematics(),
   swerveSubsystem.getHeading(), swerveSubsystem.getSwerveModulePositions(), 
   new Pose2d(), 
   stateStdDevs, 
   visionMeasurementStdDevs);

} 

@Override
public void periodic() {

  var pipelineResult = camera.getLatestResult();
  var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimeStamp && pipelineResult.hasTargets()) {
      previousPipelineTimeStamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(robotToCam);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);        
      }
    }
  
  photonPoseEstimator.update();

  poseEstimator.update(
    swerveSubsystem.getHeading(),
    swerveSubsystem.getSwerveModulePositions());
  
}

public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  return photonPoseEstimator.update();
}
  
  
public int getPoseViaTag(){
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      return target.getFiducialId();
    } else {
      return 99;
    }
  }



public void takePict(){
  camera.takeOutputSnapshot();
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



  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {
    //visionSim.update(m_swerveSubsystem.getPose());
  }


/*
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
*/
}
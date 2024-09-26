package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

// 6 inches up, 6 inches to the side for testing

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;
  private final SwerveSubsystem swerveSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator; 
  
  private final Field2d field2d = new Field2d();

  private double previousPipelineTimeStamp = 0;

// VisionSubsystem constructor
public VisionSubsystem(PhotonCamera camera, SwerveSubsystem swerveSubsystem) {
 
  this.camera = camera;
  this.swerveSubsystem = swerveSubsystem;
  var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  layout.setOrigin( OriginPosition.kBlueAllianceWallRightSide); 
  this.aprilTagFieldLayout = layout;

  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  poseEstimator = new SwerveDrivePoseEstimator(
   swerveSubsystem.getKinematics() ,
   swerveSubsystem.getHeading(),
    swerveSubsystem.getSwerveModulePositions(),
     new Pose2d(),
     stateStdDevs,
     visionMeasurementStdDevs);

  tab.addString("Pose", this::getFomattedPose).withPosition(0,0).withSize(2,0);
  tab.add("Field-Check", field2d).withPosition(2, 0).withSize(6, 4);

  
} 

@Override
public void periodic() {
  // Update pose estimator with best visible target
  PhotonPipelineResult pipelineResult = camera.getLatestResult();
  double resultTimestamp = pipelineResult.getTimestampSeconds();
  if (resultTimestamp != previousPipelineTimeStamp && pipelineResult.hasTargets()) {
    previousPipelineTimeStamp = resultTimestamp;
    PhotonTrackedTarget target = pipelineResult.getBestTarget();
    int fiducialId = target.getFiducialId();
    // Get the tag pose from the filed layout - will be null if layout failed to load.
    Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
    if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && tagPose.isPresent()) {
      Pose3d targetPose = tagPose.get();
      Transform3d camToTarget = target.getBestCameraToTarget();
      Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

      Pose3d visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);

      poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
    }
  } 
  //Update pose estimator
  poseEstimator.update(
    swerveSubsystem.getHeading(),
    swerveSubsystem.getSwerveModulePositions());
  field2d.setRobotPose(getCurrentPose());
}


private String getFomattedPose() {
  var pose = getCurrentPose();
  return String.format("(%.2f, %.2f) %.2f degrees", 
      pose.getX(), 
      pose.getY(),
      pose.getRotation().getDegrees());
}

public Pose2d getCurrentPose() {
  return poseEstimator.getEstimatedPosition();
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
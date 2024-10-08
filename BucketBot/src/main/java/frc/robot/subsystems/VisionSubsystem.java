package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// 6 inches up, 6 inches to the side for testing

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;

  public PhotonPoseEstimator photonPoseEstimator;

  private AprilTagFieldLayout layout;
  
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(10));

  private final Field2d field2d_Vis = new Field2d();

   private double previousPipelineTimeStamp = 0;

// VisionSubsystem constructor
public VisionSubsystem() {
  // Load the camera
  camera = new PhotonCamera("Logitech_Webcam_C930e");
//Load field layout
  //AprilTagFieldLayout layout =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
  try {
        layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        e.printStackTrace();
    }
  
  System.out.println(this.layout.getFieldLength());
//  AprilTagFieldLayout layout = new AprilTagFieldLayout(layout_test);
  layout.setOrigin( OriginPosition.kBlueAllianceWallRightSide); 
    // Create Vision Smart Dashboard
  SmartDashboard.putData("Vision_Field",field2d_Vis);
  // Construct PhotonPoseEstimator
  //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

  // Adding Fake ID to test posing
  SmartDashboard.putNumber("Fake ID", 3);

} 

@Override
public void periodic() {

  var pipelineResult = camera.getLatestResult();
  var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimeStamp && pipelineResult.hasTargets()) {
      previousPipelineTimeStamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      //var fiducialId = 3; //SmartDashboard.getNumber("Fake ID", 1); 
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = this.layout == null ? Optional.empty() : this.layout.getTagPose(fiducialId);
      System.out.println("field length is: " + this.layout.getFieldLength());
      System.out.println("ID is: " + fiducialId);
      System.out.println("Tag Y is: " + tagPose.get().getY());
      if (target.getPoseAmbiguity() <= .2 &&
      fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT);
        SmartDashboard.putNumber("Vision_x", visionMeasurement.getX());
        SmartDashboard.putNumber("Vision_y", visionMeasurement.getY());
        field2d_Vis.setRobotPose(visionMeasurement.toPose2d());
        SmartDashboard.putNumber("Target_x", targetPose.getX());
        SmartDashboard.putNumber("Target_y", targetPose.getY());
        //field2d_Vis.setRobotPose(targetPose.toPose2d()); 
        if (fiducialId == 4 || fiducialId == 7) {
          boolean shoot = this.shootPose(visionMeasurement);
        }
      } 
    }
  
  
  
}

public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  this.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  return photonPoseEstimator.update();
}
  
public boolean shootPose(Pose3d visionMeasurement){
double distance = visionMeasurement.getX();
double angle = visionMeasurement.getRotation().getAngle();
boolean distGood = false;
boolean angleGood = false;
if (angle > Units.degreesToRadians(57) && angle < Units.degreesToRadians(237)){
   angleGood = true;
} 
if (distance < Units.inchesToMeters(65)){
  distGood = true;
} 
return distGood && angleGood;
}

public Pose2d getPoseViaTag(){
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      return this.photonPoseEstimator.getReferencePose().toPose2d();
    } else {
      return new Pose2d(10,10, Rotation2d.fromDegrees(180));
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
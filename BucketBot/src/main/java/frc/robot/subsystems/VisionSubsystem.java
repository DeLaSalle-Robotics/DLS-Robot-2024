package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import javax.swing.text.StyleContext.SmallAttributeSet;

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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private int currentTag;

  private Transform3d currentPose;
  private Pose2d robotPose;
  private int targetID;
  private double targetYaw;
  private double targetDist;
  private boolean targetVisible;
  private boolean RedAlliance;
  private boolean verbose;
  
  //NetworkTable publishers
  BooleanPublisher InZonePub;
  BooleanPublisher OnTargetPub;
  DoublePublisher TargetYawPub;

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
  this.verbose = false;

  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("datatable");
  InZonePub = table.getBooleanTopic("InZone").publish();
  OnTargetPub = table.getBooleanTopic("OnTarget").publish();
  TargetYawPub = table.getDoubleTopic("Target_Yaw").publish();
  //Find out what alliance we are to allow setting of the speaker target.
  var alliance = DriverStation.getAlliance();
  this.RedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  if (RedAlliance) {
    targetID = 7;
  } else {
    targetID = 4;
  }
} 

@Override
public void periodic() {

  var pipelineResult = camera.getLatestResult();
  var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != this.previousPipelineTimeStamp && pipelineResult.hasTargets()) {
      this.previousPipelineTimeStamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      //var fiducialId = 3; //SmartDashboard.getNumber("Fake ID", 1); 
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = this.layout == null ? Optional.empty() : this.layout.getTagPose(fiducialId);
      if (this.verbose) {
        System.out.println("field length is: " + this.layout.getFieldLength());
        System.out.println("ID is: " + fiducialId);
        System.out.println("Tag Y is: " + tagPose.get().getY());
      }
      
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT);
        SmartDashboard.putNumber("Vision_x", visionMeasurement.getX());
        SmartDashboard.putNumber("Vision_y", visionMeasurement.getY());
        field2d_Vis.setRobotPose(visionMeasurement.toPose2d());
        this.robotPose = visionMeasurement.toPose2d();
        SmartDashboard.putNumber("Target_x", targetPose.getX());
        SmartDashboard.putNumber("Target_y", targetPose.getY());
        //Should report the angle in radians with 0 being the Blue Alliance Wall
        SmartDashboard.putNumber("Robot_Angle",Units.radiansToDegrees(visionMeasurement.getRotation().getAngle() - Math.PI));
        this.currentPose = camToTarget;
        this.currentTag = fiducialId;
        
        for (var tmpTarget : pipelineResult.getTargets()) {

          if (tmpTarget.getFiducialId() == 4) {

              // Found target Tag found, record its information

              this.targetYaw = tmpTarget.getYaw();
              TargetYawPub.set(this.targetYaw);
              double targetx = tmpTarget.getBestCameraToTarget().getX();
              double targety = tmpTarget.getBestCameraToTarget().getY();
              this.targetDist = Math.sqrt((targetx * targetx) + (targety * targety));
              SmartDashboard.putNumber("TargetDist", this.targetDist);
              this.targetVisible = true;
              SmartDashboard.putNumber("TARGET X",tmpTarget.getBestCameraToTarget().getX());
              SmartDashboard.putNumber("TARGET Y",tmpTarget.getBestCameraToTarget().getY());
              SmartDashboard.putNumber("Target_ID", currentTag);
              //field2d_Vis.setRobotPose(targetPose.toPose2d()); 
            } 
          }
        }
      }
      if(pipelineResult.hasTargets()){
        InZonePub.set(this.InZone());
        OnTargetPub.set(this.OnTarget());
        SmartDashboard.putNumber("Target Yaw", this.targetYaw);
      }
      else{
        InZonePub.set(false);
        OnTargetPub.set(false);
        
      }
}

public Pose2d getRobotPose() {
  return this.robotPose;
}
public double getRobotPoseTime(){
  return this.previousPipelineTimeStamp;
}

public boolean InZone(){
  if (this.verbose) {
    System.out.println("In Zone Calc:" + Units.radiansToDegrees(Math.abs(this.currentPose.getRotation().getAngle() )));}
  boolean distGood = false;
  boolean angleGood = false;
    
  
  try {
    if (this.targetYaw > -45 && this.targetYaw < 45){
      angleGood = true;
    } 
    if (this.targetDist < 2.0){
      distGood = true;
    } 
    }
   catch (NullPointerException e) {
    // TODO: handle exception
  }
  return (this.targetVisible) && (distGood && angleGood);
}

public boolean OnTarget(){
  if (this.verbose) {
    System.out.println("On Target Calc:" + Units.radiansToDegrees(Math.abs(this.currentPose.getRotation().getAngle() )));}
  boolean distGood = false;
  boolean angleGood = false;
    
  try {
    if (this.targetYaw > -10 && this.targetYaw < 10){
      angleGood = true;
    } 
    if (this.targetDist < 2.1 && this.targetDist > 1.75){
      distGood = true;
    } 
    }
   catch (NullPointerException e) {
    // TODO: handle exception
  }

  return (this.targetVisible) && (distGood && angleGood);
}

public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  this.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  return photonPoseEstimator.update();
}
  
public boolean shootPose(Transform3d visionMeasurement){
    try {
      double distance = visionMeasurement.getX();
    double angle = visionMeasurement.getRotation().getAngle() ;
    boolean distGood = false;
    boolean angleGood = false;
    if (angle > Units.degreesToRadians(57) && angle < Units.degreesToRadians(237)){
      angleGood = true;
    } 
    if (distance < Units.inchesToMeters(65)){
      distGood = true;
    } 
    return distGood && angleGood;}
    catch (NullPointerException a) {
      return false;
    }
}



public double getPoseViaTag(){
    return this.currentPose.getRotation().getAngle();
  }



public void takePict(){
  camera.takeOutputSnapshot();
}
public double getRotationToTarget(){
  return this.currentPose.getRotation().getAngle();
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
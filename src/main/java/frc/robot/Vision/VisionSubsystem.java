package frc.robot.Vision;

import static frc.robot.Constants.FieldConstants.aprilTags;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Vision.TimestampedVisionUpdate;
import frc.robot.Utils.GeomUtil;

public class VisionSubsystem extends SubsystemBase{
  private PhotonCamera[] cameras;

  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

  private List<TimestampedVisionUpdate> visionUpdates;

  private final Pose3d[] cameraPoses = 
    new Pose3d[]{
      new Pose3d(0, 0, 0, new Rotation3d())
    };
  public void setDataInterfaces(Supplier<Pose2d> poseSupplier){
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic(){
    Pose2d currentpose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++){
      Pose2d currentPose = poseSupplier.get();
      Pose3d cameraPose;
      Pose2d robotPose;
      List<Pose3d> tagPose3ds = new ArrayList<>();
      PhotonPipelineResult unprocessedResult = cameras[instanceIndex].getLatestResult();

      double singleTagAdjustment = 1.0;

      if(!unprocessedResult.hasTargets()){
        continue;
      }

      double timestamp = unprocessedResult.getTimestampSeconds();

      boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().estimatedPose.isPresent;

      if (shouldUseMultiTag) {
        cameraPose = 
          GeomUtil.transform3dToPose3d(unprocessedResult.getMultiTagResult().estimatedPose.best);
        
        robotPose = cameraPose
          .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
          .toPose2d();
        for (int id : unprocessedResult.getMultiTagResult().fiducialIDsUsed){
          tagPose3ds.add(aprilTags.getTagPose(id).get());
        }
      }
      else{
        PhotonTrackedTarget target = unprocessedResult.targets.get(0);
        Pose3d tagPos = aprilTags.getTagPose(target.getFiducialId()).get();
        Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
        Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
        Pose2d robotPose0 = cameraPose0
          .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
          .toPose2d();
        Pose2d robotPose1 = cameraPose1
          .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
          .toPose2d();
        
        double projectionError = target.getPoseAmbiguity();

        if (projectionError < 0.15){
          robotPose = robotPose0;
          cameraPose = cameraPose0;
        }
        else if (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
          < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())){
            cameraPose = cameraPose0;
            robotPose = robotPose0;
        }
        else{
          robotPose = robotPose1;
          cameraPose = cameraPose1;
        }
        tagPose3ds.add(tagPos);
        
      }

      
    }
  }
}

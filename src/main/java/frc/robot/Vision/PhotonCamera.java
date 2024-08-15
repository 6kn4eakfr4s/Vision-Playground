package frc.robot.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCamera implements AutoCloseable{

  private final NetworkTable cameraTable;
  public PacketSubscriber<PhotonPipelineResult> resultSubscriber;

  private static final String rootTableName = "photonvision";

  public double lastCallTime = 0.0;
  public double lastCaptureTime = 0.0;
  public double lastCallTimeTwo = 0.0;

  @Override
  public void close(){
    resultSubscriber.close();
  }

  public PhotonCamera (NetworkTableInstance ntInstance, String cameraName) {
    NetworkTable rootNTTable = ntInstance.getTable(rootTableName);
    cameraTable = rootNTTable.getSubTable(cameraName);
    RawSubscriber rawBytesSubscriber =
      cameraTable
        .getRawTopic("rawBytes")
        .subscribe("rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
    resultSubscriber = 
      new PacketSubscriber<PhotonPipelineResult>(
        rawBytesSubscriber, PhotonPipelineResult.serde, new PhotonPipelineResult());
  }

  public PhotonPipelineResult getLatestResult() {
    PhotonPipelineResult results = resultSubscriber.get();
    double currentCallTime = Timer.getFPGATimestamp();
    if(lastCallTime == 0.0){
      lastCallTime = currentCallTime;
    }
    else{
      double captureTime = results.getTimestampSeconds();
      lastCallTime = lastCallTime + (captureTime - lastCaptureTime);
      lastCaptureTime = captureTime;
      
      lastCallTime = MathUtil.clamp(lastCallTime, lastCallTimeTwo, currentCallTime);
      //don't know why not put cur
    }
    results.setTimestampSeconds(lastCallTime);

    lastCallTimeTwo = lastCallTime;
    lastCallTime = currentCallTime;

    return results;
  }
}

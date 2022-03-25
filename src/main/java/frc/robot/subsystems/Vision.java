package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import java.util.Optional;

public class Vision {
  private NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  private Optional<VisionResult> latestResult = Optional.empty();

  public Vision() {
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  public Optional<VisionResult> getLatestResult() {
    return latestResult;
  }

  // @Override
  public void periodic() {
    this.latestResult = Optional.ofNullable(getLatest());
  }

  protected VisionResult getLatest() {
    if (networkTable.getEntry("tv").getDouble(0) > 0) {
      // have a target!
      double tx = networkTable.getEntry("tx").getDouble(0);
      double ty = networkTable.getEntry("ty").getDouble(0);
      double tl = networkTable.getEntry("tl").getDouble(0);

      return new VisionResult() {
        @Override
        public double getLatencyMillis() {
          return tl;
        }

        @Override
        public double getDistance() {
          double distance = HEIGHT_FROM_CAMERA_TO_GOAL / Math.tan(Math.toRadians(CAMERA_TILT + ty));
          return distance;
        }

        @Override
        public double getYaw() {
          return tx;
        }
      };
    }
    return null;
  }

  public void setMode(Mode mode) {}

  public void setPipeline(int pipeline) {
    networkTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void takePicture() {
    networkTable.getEntry("snapshot").setNumber(1);
  }

  public void setStream(int stream) {
    int s = MathUtil.clamp(stream, 0, 3);
    networkTable.getEntry("stream").setNumber(s);
  }

  public void toggleLED() {
    var lMode = networkTable.getEntry("ledMode");
    lMode.setNumber((lMode.getNumber(0).intValue() > 0) ? 1 : 0);
  }

  public enum Mode {
    Target,
    Driver
  }

  public interface VisionResult {
    // default Pose2d getCameraPose() { return null; }
    double getLatencyMillis();

    double getDistance();
    // Transform2d getCameraToTarget();
    double getYaw();
  }
}

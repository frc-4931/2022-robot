package frc.robot;

import static frc.robot.Constants.PoseConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Vision;

public class Pose2dHandler {
  private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(0.01);
  private Matrix<N3, N1> visionMeasurementStdDevs =
      VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  private MecanumDrivePoseEstimator poseEstimator;
  private IMU imu;
  private Vision vision;

  public Pose2dHandler(IMU imu, Vision vision) {
    // set to [0,0], 0deg and rely on the real initial pose to be set from the autonomous
    var initialPose = new Pose2d();
    this.imu = imu;
    this.vision = vision;
    poseEstimator =
        new MecanumDrivePoseEstimator(
            imu.getRotation2d(),
            initialPose,
            DRIVE_KINEMATICS,
            stateStdDevs,
            localMeasurementStdDevs,
            visionMeasurementStdDevs);
  }

  public void update(MecanumDriveWheelSpeeds wheelSpeeds) {
    poseEstimator.update(imu.getRotation2d(), wheelSpeeds);

    var result = vision.getLatestResult();
    // result.ifPresent(vr -> {
    //   double imageCaptureTime = Timer.getFPGATimestamp() - vr.getLatencyMillis();
    //   Transform2d camToTargetTrans = vr.getCameraToTarget();
    //   Pose2d camPose = Constants.kTargetPose.transformBy(camToTargetTrans.inverse());
    //   poseEstimator.addVisionMeasurement(
    //                 camPose.transformBy(CAMERA_TO_ROBOT), imageCaptureTime);
    // });

  }

  public void resetToPose(Pose2d pose) {
    imu.resetAngle(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(pose, imu.getRotation2d());
  }

  public Pose2d getPoseEst() {
    return poseEstimator.getEstimatedPosition();
  }
}

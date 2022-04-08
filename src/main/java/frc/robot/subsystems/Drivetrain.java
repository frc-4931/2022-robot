package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pose2dHandler;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax motorFrontLeft;
  private CANSparkMax motorFrontRight;
  private CANSparkMax motorBackLeft;
  private CANSparkMax motorBackRight;

  // private Pose2d pose;

  private MecanumDrive mecanumDrive;
  private Pose2dHandler pose2dHandler;
  // private MecanumDriveOdometry mecanumDriveOdometry;
  // private Field2d field2d;

  public Drivetrain(Pose2dHandler pose2dHandler) {
    motorFrontLeft = FRONT_LEFT.createMotor();
    motorFrontRight = FRONT_RIGHT.createMotor();
    motorBackLeft = REAR_LEFT.createMotor();
    motorBackRight = REAR_RIGHT.createMotor();

    mecanumDrive = new MecanumDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
    mecanumDrive.setDeadband(0.06);
    this.pose2dHandler = pose2dHandler;
    // mecanumDrive.setMaxOutput(0.4);

    // mecanumDriveOdometry = new MecanumDriveOdometry(DRIVE_KINEMATICS, pigeon.getRotation2d());
  }

  // public void setInitialPose(Pose2d initialPose) {
  //   // mecanumDriveOdometry.resetPosition(pose, pigeon.getRotation2d());
  //   // pose = mecanumDriveOdometry.getPoseMeters();
  //   field2d.setRobotPose(mecanumDriveOdometry.getPoseMeters());
  // }

  public void periodicDrivetrain() {
    // Get my wheel speeds
    var wheelSpeeds =
        new MecanumDriveWheelSpeeds(
            motorFrontLeft.getEncoder().getVelocity(), motorFrontRight.getEncoder().getVelocity(),
            motorBackLeft.getEncoder().getVelocity(), motorBackRight.getEncoder().getVelocity());

    pose2dHandler.update(wheelSpeeds);

    SmartDashboard.putNumber("LeftFrontVelocity", wheelSpeeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("RightFrontVelocity", wheelSpeeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("LeftRearVelocity", wheelSpeeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("RightRearVelocity", wheelSpeeds.rearRightMetersPerSecond);
    // SmartDashboard.putData("pig", pigeon);

    log();
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
  }

  // public Pose2d getPose() {
  //   return pose;
  // }

  // public void toggleRobotOriented() {
  //   robotOriented = !robotOriented;
  // }

  // public void toggleDrivingDirection() {
  //   driveMultiplier *= -1;
  // }

  // public void setDriveMultiplier(double multipler) {
  //   driveMultiplier = Math.copySign(multipler, driveMultiplier);
  // }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("zRotation", zRotation);
    SmartDashboard.putNumber("compassHeading", gyroAngle);
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  public void drivePolar(double magnitude, double angle, double zRotation) {
    mecanumDrive.drivePolar(magnitude, angle, zRotation);
  }

  public void setDriveSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    motorFrontLeft
        .getPIDController()
        .setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kSmartMotion, 0);
    motorFrontRight
        .getPIDController()
        .setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kSmartMotion, 0);
    motorBackLeft
        .getPIDController()
        .setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kSmartMotion, 0);
    motorBackRight
        .getPIDController()
        .setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kSmartMotion, 0);
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

  public void log() {
    SmartDashboard.putNumber("Front Left Motor Speed", motorFrontLeft.get());
    SmartDashboard.putNumber("Back Left Motor Speed", motorBackLeft.get());
    SmartDashboard.putNumber("Front Right Motor Speed", motorFrontRight.get());
    SmartDashboard.putNumber("Back Right Motor Speed", motorBackRight.get());
  }
}

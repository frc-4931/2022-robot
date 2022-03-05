package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  CANSparkMax motorFrontLeft;
  CANSparkMax motorFrontRight;
  CANSparkMax motorBackLeft;
  CANSparkMax motorBackRight;
  // WPI_TalonSRX pigeonMotorController;

  private WPI_PigeonIMU pigeon;
  private Pose2d pose;

  private MecanumDrive mecanumDrive;
  private MecanumDriveOdometry mecanumDriveOdometry;
  private Field2d field2d;

  private boolean robotOriented = true;

  public Drivetrain(Field2d field2d) {
    this.field2d = field2d;

    motorFrontLeft = FRONT_LEFT.createMotor();
    motorFrontRight = FRONT_RIGHT.createMotor();
    motorBackLeft = REAR_LEFT.createMotor();
    motorBackRight = REAR_RIGHT.createMotor();

    mecanumDrive = new MecanumDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
    mecanumDrive.setDeadband(0.06);
    // mecanumDrive.setMaxOutput(0.4);

    pigeon = new WPI_PigeonIMU(new WPI_TalonSRX(PIGEON_MOTOR_PORT));
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
    zero();
    // pigeon.configTemperatureDompensationEnable(true, 0);

    mecanumDriveOdometry = new MecanumDriveOdometry(DRIVE_KINEMATICS, pigeon.getRotation2d());
  }

  public void setInitialPose(Pose2d initialPose) {
    mecanumDriveOdometry.resetPosition(pose, pigeon.getRotation2d());
    pose = mecanumDriveOdometry.getPoseMeters();
    field2d.setRobotPose(mecanumDriveOdometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    // Get my wheel speeds
    var wheelSpeeds =
        new MecanumDriveWheelSpeeds(
            motorFrontLeft.getEncoder().getVelocity(), motorFrontRight.getEncoder().getVelocity(),
            motorBackLeft.getEncoder().getVelocity(), motorBackRight.getEncoder().getVelocity());

    var gyroAngle = pigeon.getRotation2d();

    // Update the pose
    pose = mecanumDriveOdometry.update(gyroAngle, wheelSpeeds);
    field2d.setRobotPose(mecanumDriveOdometry.getPoseMeters());

    SmartDashboard.putNumber("LeftFrontVelocity", wheelSpeeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("RightFrontVelocity", wheelSpeeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("LeftRearVelocity", wheelSpeeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("RightRearVelocity", wheelSpeeds.rearRightMetersPerSecond);
    SmartDashboard.putData("pig", pigeon);
    SmartDashboard.putBoolean("robotOriented", robotOriented);

    log();
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
  }

  public Pose2d getPose() {
    return pose;
  }

  public void toggleRobotOriented() {
    robotOriented = !robotOriented;
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void driveCartesianFieldOriented(double ySpeed, double xSpeed, double zRotation) {
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("zRotation", zRotation);
    SmartDashboard.putNumber("compassHeading", getAngleContinuous());
    if (robotOriented == true) {
      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
    } else {
      // System.out.printf(
      //     "drive y: %d x: %d z: %d heading: %d", ySpeed, xSpeed, zRotation, getCompassHeading());
      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, getAngleContinuous());
    }
  }

  public void drivePolar(double magnitude, double angle, double zRotation) {
    mecanumDrive.drivePolar(magnitude, angle, zRotation);
  }

  public void setDriveSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    motorFrontLeft
        .getPIDController()
        .setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kSmartVelocity, 0);
    motorFrontRight
        .getPIDController()
        .setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kSmartVelocity, 0);
    motorBackLeft
        .getPIDController()
        .setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kSmartVelocity, 0);
    motorBackRight
        .getPIDController()
        .setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kSmartVelocity, 0);
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

  public void log() {
    SmartDashboard.putNumber("Front Left Motor Speed", motorFrontLeft.get());
    SmartDashboard.putNumber("Back Left Motor Speed", motorBackLeft.get());
    SmartDashboard.putNumber("Front Right Motor Speed", motorFrontRight.get());
    SmartDashboard.putNumber("Back Right Motor Speed", motorBackRight.get());
    SmartDashboard.putNumber("Angle", getAngle());
    SmartDashboard.putNumber("Angle Continuous", getAngleContinuous());
    SmartDashboard.putNumber("Compass Angle Absolute", getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Compass Angle", getCompassHeading());
    SmartDashboard.putNumber("Pigeon Temp", pigeon.getTemp());
    var stat = getGeneralStatus();
    SmartDashboard.putNumber("Compass Cal Error", stat.calibrationError);
    SmartDashboard.putBoolean("Compass Calibrating", stat.bCalIsBooting);

    if (SmartDashboard.getBoolean("Reset Gyro", false)) {
      zero();
      SmartDashboard.putBoolean("Reset Gyro", false);
    }

    if (SmartDashboard.getBoolean("Reset Comp", false)) {
      resetCompass();
      SmartDashboard.putBoolean("Reset Comp", false);
    }
  }

  public double getAngle() {
    double angle = pigeon.getFusedHeading() % 360;
    double out = (angle < -180) ? angle + 360 : angle;
    return (out > 180) ? out - 360 : out;
  }

  public double getAngleContinuous() {
    return -pigeon.getFusedHeading();
  }

  public double getAbsoluteCompassHeading() {
    return pigeon.getAbsoluteCompassHeading();
  }

  public double getCompassHeading() {
    return pigeon.getCompassHeading();
  }

  public void resetCompass() {
    pigeon.enterCalibrationMode(CalibrationMode.Accelerometer);
  }

  public GeneralStatus getGeneralStatus() {
    GeneralStatus status = new GeneralStatus();
    pigeon.getGeneralStatus(status);
    return status;
  }

  public double getTiltAcceleration() {
    double[] rawGyroData = new double[3];
    pigeon.getRawGyro(rawGyroData);
    return rawGyroData[0];
  }

  public void zero() {
    pigeon.setFusedHeading(0);
  }
}

/**
 * TITLE: DriveTrain 
 * PURPOSE: create a drive train
 * TODO: set default command for init
 * INFO: motor will be NEO and controler will be CANSparkMax
 */
package frc.robot.subsystems;

// imports for Spark Max

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax motorFrontLeft;
  private CANSparkMax motorFrontRight;
  private CANSparkMax motorBackLeft;
  private CANSparkMax motorBackRight;
  private WPI_TalonSRX pigeonMotorController;
  private static final int FrontLeftDeviceID = 1;
  private static final int FrontRightDeviceID = 2;
  private static final int BackLeftDeviceID = 3;
  private static final int BackRightDeviceID = 4;
  private PigeonIMU pigeon;

  private MecanumDrive mecanumDrive;

  public Drivetrain() {
    motorFrontLeft = new CANSparkMax(FrontLeftDeviceID, MotorType.kBrushless);
    motorFrontRight = new CANSparkMax(FrontRightDeviceID, MotorType.kBrushless);
    motorBackLeft = new CANSparkMax(BackLeftDeviceID, MotorType.kBrushless);
    motorBackRight = new CANSparkMax(BackRightDeviceID, MotorType.kBrushless);

    mecanumDrive = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

    mecanumDrive.setDeadband(0.05);

    pigeonMotorController = new WPI_TalonSRX(0);
    pigeon = new PigeonIMU(pigeonMotorController);

    // pigeon.configTemperatureDompensationEnable(true, 0);
  }

  public void initDefaultCommand() {
  

  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void driveCartesianFieldOriented(
      double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  public void drivePolar(double magnitude, double angle, double zRotation) {
    mecanumDrive.drivePolar(magnitude, angle, zRotation);
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

//this will set up the smartDashboard display
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
//these methods are using the pigeon gyro sensor
  public double getAngle() {
    double angle = -pigeon.getFusedHeading() % 360;
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

  public void reset() {
    pigeon.setFusedHeading(0);
  }
}

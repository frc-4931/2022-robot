package frc.robot.subsystems;

import static frc.robot.Constants.SensorConstants.PIGEON_MOTOR_PORT;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMU implements Gyro {
  public enum AccelerationAngles {
    X,
    Y,
    Z
  }

  private WPI_PigeonIMU pigeon;

  public IMU() {
    pigeon = new WPI_PigeonIMU(new WPI_TalonSRX(PIGEON_MOTOR_PORT));
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
    reset();
    // pigeon.configTemperatureDompensationEnable(true, 0);
  }

  @Override
  public double getAngle() {
    return pigeon.getAngle();
  }

  public double getModedAngle() {
    double angle = getAngle() % 360;
    double out = (angle < -180) ? angle + 360 : angle;
    return (out > 180) ? out - 360 : out;
  }

  public double[] getAccelerometerAngles() {
    double[] tiltAngles = new double[3];
    pigeon.getAccelerometerAngles(tiltAngles);
    return tiltAngles;
  }

  // public double getAngleContinuous() {
  //   return -pigeon.getFusedHeading();
  // }

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

  public double getTiltSpeed() {
    double[] rawGyroData = new double[3];
    pigeon.getRawGyro(rawGyroData);
    return rawGyroData[0];
  }

  public void log() {
    SmartDashboard.putNumber("Angle", getModedAngle());
    SmartDashboard.putNumber("Angle Continuous", getAngle());
    SmartDashboard.putNumber("Compass Angle Absolute", getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Compass Angle", getCompassHeading());
    SmartDashboard.putNumber("Pigeon Temp", pigeon.getTemp());
    var stat = getGeneralStatus();
    SmartDashboard.putNumber("Compass Cal Error", stat.calibrationError);
    SmartDashboard.putBoolean("Compass Calibrating", stat.bCalIsBooting);

    if (SmartDashboard.getBoolean("Reset Gyro", false)) {
      reset();
      SmartDashboard.putBoolean("Reset Gyro", false);
    }

    if (SmartDashboard.getBoolean("Reset Comp", false)) {
      resetCompass();
      SmartDashboard.putBoolean("Reset Comp", false);
    }
  }

  @Override
  public void close() throws Exception {
    pigeon.close();
  }

  @Override
  public void calibrate() {
    pigeon.calibrate();
  }

  @Override
  public void reset() {
    pigeon.reset();
  }

  public void resetAngle(double angle) {
    pigeon.setFusedHeading(-angle);
  }

  @Override
  public double getRate() {
    return pigeon.getRate();
  }
}

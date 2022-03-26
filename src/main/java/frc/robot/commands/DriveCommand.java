package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class DriveCommand extends CommandBase {
  public enum ForwardDirection {
    Intake,
    Shooter
  }

  private Drivetrain drivetrain;
  private Vision vision;
  private Gyro gyro;
  private BiConsumer<Double, Double> rumbler;
  private int driverNotify = -1;
  private Supplier<Double> yAxisSupplier;
  private Supplier<Double> xAxisSupplier;
  private Supplier<Double> zAxisSupplier;
  private Supplier<Double> multiplierSupplier;
  private boolean lockAngle = false;
  private boolean fieldOriented = true;
  private double driveMultiplier = 1;
  private PIDController turnController;

  public DriveCommand(
      Drivetrain drivetrain,
      Supplier<Double> yAxisSupplier,
      Supplier<Double> xAxisSupplier,
      Supplier<Double> zAxisSupplier,
      Supplier<Double> multiplierSupplier,
      Vision vision,
      Gyro gyro) {
    this(drivetrain, yAxisSupplier, xAxisSupplier, zAxisSupplier, multiplierSupplier, vision, gyro, (l, r) -> {});
  }

  public DriveCommand(
      Drivetrain drivetrain,
      Supplier<Double> yAxisSupplier,
      Supplier<Double> xAxisSupplier,
      Supplier<Double> zAxisSupplier,
      Supplier<Double> multiplierSupplier,
      Vision vision,
      Gyro gyro,
      BiConsumer<Double, Double> rumble) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.yAxisSupplier = yAxisSupplier;
    this.xAxisSupplier = xAxisSupplier;
    this.zAxisSupplier = zAxisSupplier;
    this.multiplierSupplier = multiplierSupplier;
    this.gyro = gyro;
    this.rumbler = rumble;
    turnController = new PIDController(0.5, 0, 0);
    setTargetLock(false);
    setForward(ForwardDirection.Intake);
    setDriveMultiplier(1);
    setFieldOriented(true);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    driveMultiplier = Math.min(1-multiplierSupplier.get(),.5);
    // if running locked, get theta from vision
    if (lockAngle) {
      var target = vision.getLatestResult();
      driverNotify = (driverNotify + 1) % NOTIFY_RATE;
      var needsDriverNotify = driverNotify == 0;
      double left = 0d;
      double right = 0d;
      if (target.isPresent()) {
        var setpoint = target.get().getYaw();
        var angle = gyro.getAngle();
        drivetrain.driveCartesian(
          driveMultiplier * yAxisSupplier.get(),
          driveMultiplier * xAxisSupplier.get(),
            turnController.calculate(angle, setpoint),
            angle);
        if (needsDriverNotify) {
          left = RUMBLE_LEFT_LOCKED;
          right = RUMBLE_RIGHT_LOCKED;
        }
      } else if (needsDriverNotify) {
        left = RUMBLE_LEFT_NO_TARGET;
        right = RUMBLE_RIGHT_NO_TARGET;
      }

      rumbler.accept(left, right);
    }
    // else get theta from joystick
    else {
      if (fieldOriented) {
        drivetrain.driveCartesian(
          driveMultiplier * yAxisSupplier.get(),  driveMultiplier * xAxisSupplier.get(), zAxisSupplier.get(), gyro.getAngle());
      } else {
        drivetrain.driveCartesian(
            driveMultiplier * yAxisSupplier.get(),
            driveMultiplier * xAxisSupplier.get(),
            zAxisSupplier.get());
      }
    }
  }

  public void toggleFieldOriented() {
    setFieldOriented(!fieldOriented);
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
    SmartDashboard.putBoolean("FieldOriented", fieldOriented);
  }

  public void toggleDriveDirection() {
    setDriveMultiplier(driveMultiplier, -1 * driveMultiplier);
  }

  public void setForward(ForwardDirection forwardDirection) {
    double dir = (forwardDirection == ForwardDirection.Intake) ? 1d : -1d;
    setDriveMultiplier(driveMultiplier, dir);
  }

  public void setDriveMultiplier(double multiplier) {
    setDriveMultiplier(MathUtil.clamp(multiplier, 0, 1), driveMultiplier);
  }

  private void setDriveMultiplier(double multiplier, double sign) {
    driveMultiplier = Math.copySign(Math.abs(multiplier), sign);

    SmartDashboard.putBoolean("FrontIsIntake", driveMultiplier > 0);
    SmartDashboard.putNumber("DriveMultiplier", Math.abs(driveMultiplier));
  }

  public void toggleTargetLock() {
    setTargetLock(!lockAngle);
  }

  public void setTargetLock(boolean targetLock) {
    lockAngle = targetLock;
    SmartDashboard.putBoolean("TargetLocked", lockAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

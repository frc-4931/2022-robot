package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.RumbleHelper;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveCommand extends CommandBase {
  public enum ForwardDirection {
    Intake,
    Shooter
  }

  private Drivetrain drivetrain;
  private Vision vision;
  private Gyro gyro;
  private Optional<RumbleHelper> rumbler;
  private Supplier<Double> yAxisSupplier;
  private Supplier<Double> xAxisSupplier;
  private Supplier<Double> zAxisSupplier;
  private Supplier<Double> multiplierSupplier;
  private boolean lockAngle = false;
  private boolean fieldOriented = true;
  private boolean intakeIsFront = true;
  // private double driveMultiplier = 1;
  private PIDController turnController;

  public DriveCommand(
      Drivetrain drivetrain,
      Supplier<Double> yAxisSupplier,
      Supplier<Double> xAxisSupplier,
      Supplier<Double> zAxisSupplier,
      Supplier<Double> multiplierSupplier,
      Vision vision,
      Gyro gyro) {
    this(
        drivetrain,
        yAxisSupplier,
        xAxisSupplier,
        zAxisSupplier,
        multiplierSupplier,
        vision,
        gyro,
        Optional.empty());
  }

  public DriveCommand(
      Drivetrain drivetrain,
      Supplier<Double> yAxisSupplier,
      Supplier<Double> xAxisSupplier,
      Supplier<Double> zAxisSupplier,
      Supplier<Double> multiplierSupplier,
      Vision vision,
      Gyro gyro,
      RumbleHelper rumble) {
    this(
        drivetrain,
        yAxisSupplier,
        xAxisSupplier,
        zAxisSupplier,
        multiplierSupplier,
        vision,
        gyro,
        Optional.of(rumble));
  }

  private DriveCommand(
      Drivetrain drivetrain,
      Supplier<Double> yAxisSupplier,
      Supplier<Double> xAxisSupplier,
      Supplier<Double> zAxisSupplier,
      Supplier<Double> multiplierSupplier,
      Vision vision,
      Gyro gyro,
      Optional<RumbleHelper> rumble) {
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
    setFieldOriented(true);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    SmartDashboard.putBoolean("FrontIsIntake", intakeIsFront);
    var multiplier = Math.max(1 - multiplierSupplier.get(), .5);
    SmartDashboard.putNumber("DriveMultiplier", multiplier);

    // if running locked, get theta from vision
    if (lockAngle) {
      var target = vision.getLatestResult();
      if (target.isPresent()) {
        var setpoint = target.get().getYaw();
        var angle = gyro.getAngle();
        drivetrain.driveCartesian(
            multiplier * yAxisSupplier.get(),
            multiplier * xAxisSupplier.get(),
            turnController.calculate(angle, setpoint),
            angle);
        rumbler.ifPresent(r -> r.rumble(RUMBLE_LEFT_LOCKED, RUMBLE_RIGHT_LOCKED, NOTIFY_RATE));

      } else {
        rumbler.ifPresent(
            r -> r.rumble(RUMBLE_LEFT_NO_TARGET, RUMBLE_RIGHT_NO_TARGET, NOTIFY_RATE));
      }
    }
    // else get theta from joystick
    else {
      if (fieldOriented) {
        drivetrain.driveCartesian(
            multiplier * yAxisSupplier.get(),
            multiplier * xAxisSupplier.get(),
            multiplier * zAxisSupplier.get(),
            gyro.getAngle());
      } else {
        var driveMultiplier = multiplier * (intakeIsFront ? 1 : -1);
        drivetrain.driveCartesian(
            driveMultiplier * yAxisSupplier.get(),
            driveMultiplier * xAxisSupplier.get(),
            multiplier * zAxisSupplier.get());
      }
    }
  }

  public void setupDriving() {
    setFieldOriented(false);
    setForward(ForwardDirection.Intake);
  }

  public void toggleFieldOriented() {
    setFieldOriented(!fieldOriented);
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
    SmartDashboard.putBoolean("FieldOriented", fieldOriented);
  }

  public void toggleDriveDirection() {
    setForward(intakeIsFront ? ForwardDirection.Shooter : ForwardDirection.Intake);
  }

  public void setForward(ForwardDirection forwardDirection) {
    intakeIsFront = forwardDirection == ForwardDirection.Intake;
  }

  // public void setDriveMultiplier(double multiplier) {
  //   setDriveMultiplier(MathUtil.clamp(multiplier, 0, 1), driveMultiplier);
  // }

  // private void setDriveMultiplier(double multiplier, double sign) {
  //   driveMultiplier = Math.copySign(Math.abs(multiplier), sign);

  //   SmartDashboard.putBoolean("FrontIsIntake", driveMultiplier > 0);
  //   SmartDashboard.putNumber("DriveMultiplier", Math.abs(driveMultiplier));
  // }

  public void toggleTargetLock() {
    setTargetLock(!lockAngle);
  }

  public void setTargetLock(boolean targetLock) {
    lockAngle = targetLock;
    if (targetLock) {
      setForward(ForwardDirection.Intake);
    }
    SmartDashboard.putBoolean("TargetLocked", lockAngle);
  }

  public void engageTargetLock() {}

  public void disengageTargetLock() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

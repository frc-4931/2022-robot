package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import java.util.List;
import lombok.Builder;
import lombok.Getter;
import lombok.Singular;

@Builder
@Getter
public class MotorConfig {
  private int canId;
  @Builder.Default private double closedLoopRampRate = 0;
  @Builder.Default private double openLoopRampRate = 0;
  @Builder.Default private MotorType type = MotorType.kBrushless;
  private SoftLimit softLimitForward;
  private SoftLimit softLimitReverse;
  @Builder.Default private IdleMode idleMode = IdleMode.kBrake;
  @Builder.Default private boolean inverted = false;
  @Builder.Default private double positionConversionFactor = 1;
  @Singular private List<PIDConfig> pidConfigs;
  private MotorConfig follower;

  public CANSparkMax createMotor() {
    return createMotor(true);
  }

  private CANSparkMax createMotor(boolean burnFlash) {
    var motor = new CANSparkMax(getCanId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setClosedLoopRampRate(getClosedLoopRampRate());
    motor.setIdleMode(getIdleMode());
    motor.setInverted(isInverted());
    motor.setOpenLoopRampRate(getOpenLoopRampRate());
    if (getSoftLimitForward() != null) {
      motor.enableSoftLimit(SoftLimitDirection.kForward, true);
      motor.setSoftLimit(SoftLimitDirection.kForward, getSoftLimitForward().getLimit());
    }
    if (getSoftLimitReverse() != null) {
      motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      motor.setSoftLimit(SoftLimitDirection.kReverse, getSoftLimitReverse().getLimit());
    }

    motor.getEncoder().setPositionConversionFactor(getPositionConversionFactor());
    motor.getEncoder().setVelocityConversionFactor(getPositionConversionFactor() / 60);
    motor.getEncoder().setPosition(0);
    
    SparkMaxPIDController pidController = motor.getPIDController();
    for (int i = 0; i < getPidConfigs().size(); i++) {
      PIDConfig pidConfig = getPidConfigs().get(i);
      pidController.setP(pidConfig.getKP(), i);
      pidController.setI(pidConfig.getKI(), i);
      pidController.setD(pidConfig.getKD(), i);
      pidController.setFF(pidConfig.getKFF(), i);
      pidController.setOutputRange(
          pidConfig.getOutputRangeLow(), pidConfig.getOutputRangeHigh(), i);
      pidController.setSmartMotionAllowedClosedLoopError(pidConfig.getAllowedClosedLoopError(), i);
      pidController.setSmartMotionMaxAccel(pidConfig.getMaxAcceleration(), i);
      pidController.setSmartMotionMaxVelocity(pidConfig.getMaxVelocity(), i);
      pidController.setSmartMotionMinOutputVelocity(pidConfig.getMinOutputVelocity(), i);
    }

    if (follower != null) {
      boolean followerInverted = follower.inverted;
      follower.inverted = this.inverted;

      var followerMotor = follower.createMotor(false);
      followerMotor.follow(motor, followerInverted);
      followerMotor.burnFlash();
    }

    if (burnFlash) {
      motor.burnFlash();
    }
    return motor;
  }

  @Builder
  @Getter
  public static final class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    @Builder.Default private double outputRangeLow = -1d;
    @Builder.Default private double outputRangeHigh = 1d;
    @Builder.Default private double allowedClosedLoopError = 0;
    private double maxAcceleration;
    private double maxVelocity;
    private double minOutputVelocity;
  }

  @Builder
  @Getter
  public static final class SoftLimit {
    private float limit;
  }
}

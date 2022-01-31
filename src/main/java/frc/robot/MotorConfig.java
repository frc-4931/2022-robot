package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import lombok.Builder;
import lombok.Getter;
import lombok.Singular;

@Builder
@Getter
public class MotorConfig {
  private int canId;
  @Builder.Default
  private double closedLoopRampRate = 0;
  @Builder.Default
  private double openLoopRampRate = 0;
  @Builder.Default
  private IdleMode idleMode = IdleMode.kBrake;
  @Builder.Default
  private boolean inverted = false;
  @Singular
  private List<PIDConfig> pidConfigs;

  public CANSparkMax createMotor() {
    var motor = new CANSparkMax(getCanId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setClosedLoopRampRate(getClosedLoopRampRate());
    motor.setIdleMode(getIdleMode());
    motor.setInverted(isInverted());
    motor.setOpenLoopRampRate(getOpenLoopRampRate());

    SparkMaxPIDController pidController = motor.getPIDController();
    for (int i = 0; i < getPidConfigs().size(); i++) {
      PIDConfig pidConfig = getPidConfigs().get(i);
      pidController.setP(pidConfig.getKP(), i);
      pidController.setI(pidConfig.getKI(), i);
      pidController.setD(pidConfig.getKD(), i);
      pidController.setFF(pidConfig.getKFF(), i);
      pidController.setOutputRange(pidConfig.getOutputRangeLow(), pidConfig.getOutputRangeHigh(), i);
      pidController.setSmartMotionAllowedClosedLoopError(pidConfig.getAllowedClosedLoopError(), i);
      pidController.setSmartMotionMaxAccel(pidConfig.getMaxAcceleration(), i);
      pidController.setSmartMotionMaxVelocity(pidConfig.getMaxVelocity(), i);
      pidController.setSmartMotionMinOutputVelocity(pidConfig.getMinOutputVelocity(), i);
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
    private double outputRangeLow;
    private double outputRangeHigh;
    private double allowedClosedLoopError;
    private double maxAcceleration;
    private double maxVelocity;
    private double minOutputVelocity;
  }
}

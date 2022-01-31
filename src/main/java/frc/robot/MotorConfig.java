package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

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

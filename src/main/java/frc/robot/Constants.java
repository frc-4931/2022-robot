// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorConfig.PIDConfig;
import frc.robot.MotorConfig.SoftLimit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final boolean USE_XBOX = true;
    public static final int DRIVER_1 = 0;
    public static final int DRIVER_2 = 1;
    public static final int DRIVER_3 = 2;

    public static final int FRONT_CAMERA = 0;
    public static final String FRONT_CAMERA_NAME = "front-camera";

    public static final double NOTIFY_RATE = 2; // rumble drive feedback once every <rate> seconds;
    public static final double RUMBLE_LEFT_LOCKED = .5;
    public static final double RUMBLE_RIGHT_LOCKED = 0d;
    public static final double RUMBLE_LEFT_NO_TARGET = .9;
    public static final double RUMBLE_RIGHT_NO_TARGET = .9;
  }

  /** Constants related to the Autonomous mode. */
  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; // TODO:
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO:
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class DriveConstants {
    private static final double OPEN_RAMP_RATE = 0.75;
    private static final double WHEEL_DIAMETER_M = Units.inchesToMeters(8);
    private static final double DRIVE_GEAR_RATIO = (70d / 14d) * (66d / 30d);
    private static final double ENCODER_POSITION_CONVERSION =
        Math.PI * WHEEL_DIAMETER_M / DRIVE_GEAR_RATIO;
    // TODO: Find actual values for these!
    private static final PIDConfig PID_DEFAULTS =
        PIDConfig.builder()
            .kP(5e-5)
            .kI(1e-6)
            .kD(0)
            .kFF(0.000156)
            .maxAcceleration(1500)
            .maxVelocity(2000)
            .outputRangeHigh(1)
            .outputRangeLow(-1)
            .allowedClosedLoopError(1)
            // .minOutputVelocity(minOutputVelocity)
            .build();
    public static final MotorConfig FRONT_LEFT =
        MotorConfig.builder()
            .canId(10)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            .build();
    public static final MotorConfig FRONT_RIGHT =
        MotorConfig.builder()
            .canId(5)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            .build();
    public static final MotorConfig REAR_LEFT =
        MotorConfig.builder()
            .canId(8)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            .build();
    public static final MotorConfig REAR_RIGHT =
        MotorConfig.builder()
            .canId(9)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            .build();
  }

  public static final class PoseConstants {
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(22.09);
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(14);

    public static final MecanumDriveKinematics DRIVE_KINEMATICS =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // Physical location of the camera on the robot, relative to the center of the
    // robot.
    public static final Transform2d CAMERA_TO_ROBOT =
        new Transform2d(
            new Translation2d(-0.25, 0), // in meters (x,y)
            new Rotation2d());

    // 4 ft. 5⅜ in diameter
    private static final double targetWidth = Units.inchesToMeters(4 * 12 + 5 + 3 / 8); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    private static final double targetHeight =
        Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    public static final double targetHeightAboveGround = Units.inchesToMeters(81.19); // meters

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos =
        Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final Pose2d kFarTargetPose =
        new Pose2d(new Translation2d(kFarTgtXPos, kFarTgtYPos), new Rotation2d(0.0));
  }

  public static final class IntakeConstants {
    public static final MotorConfig INTAKE_MOTOR =
        MotorConfig.builder()
            .canId(6)
            .idleMode(IdleMode.kCoast)
            .pidConfig(
                PIDConfig.builder()
                    .kP(0.74646)
                    .kI(0)
                    .kD(0)
                    .kFF(0.000156)
                    // .maxAcceleration(maxAcceleration)
                    // .maxVelocity(maxVelocity)
                    // .outputRangeHigh(1)
                    // .outputRangeLow(-1)
                    // .minOutputVelocity(minOutputVelocity)
                    .build())
            .build();

    public static final MotorConfig INTAKE_LIFT_MOTOR =
        MotorConfig.builder()
            .canId(12)
            .idleMode(IdleMode.kBrake)
            // .inverted(true)
            .pidConfig(
                PIDConfig.builder()
                    .kP(0.00005)
                    .kI(0.000001)
                    .kD(0)
                    .kFF(0.000156)
                    .maxVelocity(4700)
                    .maxAcceleration(8000)
                    .allowedClosedLoopError(0)
                    .build())
            .pidConfig(
                PIDConfig.builder()
                    .kP(0.00005)
                    .kI(0.000001)
                    .kD(0)
                    .kFF(0.000156)
                    .maxVelocity(5700)
                    .maxAcceleration(2500)
                    .allowedClosedLoopError(0)
                    .build())
            .softLimitForward(SoftLimit.builder().limit(29).build())
            .softLimitReverse(SoftLimit.builder().limit(-.5f).build())
            .openLoopRampRate(5)
            // .m
            .build();
    public static final double UP_POSITION = 0;
    public static final double DOWN_POSITION = 29;
    public static final double LIFT_THRESHOLD = .5 * DOWN_POSITION;
    public static final double WAIT_BEFORE_STARTIING = 1;
    public static final double SPEED = 5700 * .90;
  }

  public static final class ElevatorConstants {
    // TODO: find these values from SimId
    private static final PIDConfig PID_DEFAULTS =
        PIDConfig.builder()
            .kP(0.67755)
            .kI(0)
            .kD(0)
            .kFF(0.000156)
            // .maxAcceleration(maxAcceleration)
            // .maxVelocity(maxVelocity)
            // .outputRangeHigh(outputRangeHigh)
            // .outputRangeLow(outputRangeLow)
            // .minOutputVelocity(minOutputVelocity)
            .build();
    public static final MotorConfig ELEVATOR_MOTOR =
        MotorConfig.builder().canId(7).idleMode(IdleMode.kCoast).pidConfig(PID_DEFAULTS).build();

    public static final double SPEED = 5700 * .35;
  }

  public static final class ShooterConstants {
    private static final PIDConfig PID_DEFAULTS =
        PIDConfig.builder()
            .kP(0.098557)
            .kI(0)
            .kD(0)
            .kFF(0.000156)
            .maxAcceleration(14000)
            .maxVelocity(5700)
            .outputRangeHigh(1)
            .outputRangeLow(-1)
            .allowedClosedLoopError(.8)
            // .minOutputVelocity(minOutputVelocity)
            .build();
    public static final MotorConfig SHOOTER_MOTOR =
        MotorConfig.builder()
            .canId(13)
            .idleMode(IdleMode.kCoast)
            .pidConfig(PID_DEFAULTS)
            // TODO: if we add a second motor
            //   .follower(
            //
            // MotorConfig.builder().canId(15).idleMode(IdleMode.kCoast).pidConfig(PID_DEFAULTS).inverted(true).build()
            //   )
            .build();
  }

  public static final class SensorConstants {
    public static final String CAMERA = "photoncamera"; // TODO:
    public static final int PIGEON_MOTOR_PORT = 2;
  }

  public static final class VisionConstants {
    public static final double HEIGHT_TO_GOAL = Units.inchesToMeters(8 * 12 + 5 + 3 / 8);
    public static final double HEIGHT_TO_CAMERA = Units.inchesToMeters(45); // FIXME:
    public static final double HEIGHT_FROM_CAMERA_TO_GOAL = HEIGHT_TO_GOAL - HEIGHT_TO_CAMERA;
    public static final double CAMERA_TILT = 20; // degrees
  }
}

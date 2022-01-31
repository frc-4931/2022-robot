// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorConfig.PIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int XBOX_PORT = 1;
  }
  

  // Spark Max CAN
  // public static final int DRIVETRAIN_FRONT_LEFT = 1;
  // public static final int DRIVETRAIN_FRONT_RIGHT = 2;
  // public static final int DRIVETRAIN_REAR_LEFT = 3;
  // public static final int DRIVETRAIN_REAR_RIGHT = 4;

  // Phoenix CAN
  

  /**
   * Constants related to the Autonomous mode.
   */
  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; //TODO:
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; //TODO:
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
    //TODO: Find actual values for these!
    private static final PIDConfig PID_DEFAULTS = PIDConfig.builder()
      .kP(0.8).kI(0).kD(0).kFF(5)
      // .maxAcceleration(maxAcceleration)
      // .maxVelocity(maxVelocity)
      // .outputRangeHigh(outputRangeHigh)
      // .outputRangeLow(outputRangeLow)
      // .minOutputVelocity(minOutputVelocity)
    .build();
    public static final MotorConfig FRONT_LEFT = MotorConfig.builder()
      .canId(0)
      .idleMode(IdleMode.kCoast)
      .pidConfig(PID_DEFAULTS)
      .build();
    public static final MotorConfig FRONT_RIGHT = MotorConfig.builder()
      .canId(1)
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .pidConfig(PID_DEFAULTS)
      .build();    
    public static final MotorConfig REAR_LEFT = MotorConfig.builder()
      .canId(2)
      .idleMode(IdleMode.kCoast)
      .pidConfig(PID_DEFAULTS)
      .build();
    public static final MotorConfig REAR_RIGHT = MotorConfig.builder()
      .canId(3)
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .pidConfig(PID_DEFAULTS)
      .build();

    public static final int PIGEON_MOTOR_PORT = 1;
    
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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(8);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // public static final SimpleMotorFeedforward kFeedforward =
    //     new SimpleMotorFeedforward(1, 0.8, 0.15);

    // // Example value only - as above, this must be tuned for your drive!
    // public static final double kPFrontLeftVel = 0.5;
    // public static final double kPRearLeftVel = 0.5;
    // public static final double kPFrontRightVel = 0.5;
    // public static final double kPRearRightVel = 0.5;
  }

}

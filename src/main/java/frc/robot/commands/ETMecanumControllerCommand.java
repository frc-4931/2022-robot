package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Edwardsville Technologies version of the <code>MecanumControllerCommand</code>. This version is
 * built to use the holonomic trajectory information available from the PathPlannerTrajectory
 * library. It also assumes that smart motor controllers are being used for the drivetrain to handle
 * PID controls.
 */
public class ETMecanumControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final Supplier<PathPlannerTrajectory> trajectorySupplier;
  private final Supplier<Pose2d> pose;
  private final MecanumDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final double maxWheelVelocityMetersPerSecond;
  private final Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds;
  private PathPlannerTrajectory trajectory;

  public ETMecanumControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    this(
        () -> trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        maxWheelVelocityMetersPerSecond,
        outputWheelSpeeds,
        requirements);
  }

  public ETMecanumControllerCommand(
      Supplier<PathPlannerTrajectory> trajectory,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {

    this.trajectorySupplier = trajectory;
    this.pose = pose;
    this.kinematics = kinematics;
    this.maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;
    this.outputWheelSpeeds = outputWheelSpeeds;

    this.controller = new HolonomicDriveController(xController, yController, thetaController);
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    trajectory = trajectorySupplier.get();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();

    var desiredState = (PathPlannerState) trajectory.sample(curTime);

    var targetChassisSpeeds =
        controller.calculate(pose.get(), desiredState, desiredState.holonomicRotation);
    var targetWheelSpeeds = kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.desaturate(maxWheelVelocityMetersPerSecond);

    // var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
    // var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
    // var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
    // var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

    // TODO: maybe log entries?

    outputWheelSpeeds.accept(targetWheelSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
